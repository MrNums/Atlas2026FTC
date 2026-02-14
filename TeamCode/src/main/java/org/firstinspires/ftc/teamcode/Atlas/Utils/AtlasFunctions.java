package org.firstinspires.ftc.teamcode.Atlas.Utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AtlasFunctions {

    public static class ShooterConfig {
        public final double ticksPerRev;
        public final double holdRpm;
        public final double closeRpm;
        public final double farRpm;

        public ShooterConfig(double ticksPerRev, double holdRpm, double closeRpm, double farRpm) {
            if (ticksPerRev <= 0) throw new IllegalArgumentException("ticksPerRev must be > 0");
            this.ticksPerRev = ticksPerRev;
            this.holdRpm = holdRpm;
            this.closeRpm = closeRpm;
            this.farRpm = farRpm;
        }
    }

    public enum RangePreset { CLOSE, FAR }
    public enum ShooterMode { OFF, HOLD, TARGET }
    public enum IntakeMode { OFF, IN, OUT }
    public enum IndexerMode { OFF, FEED, HOLD, REVERSE }
    public enum ArtifactType { NONE, PURPLE, GREEN, UNKNOWN }

    // ---------------- HARDWARE ----------------
    private final AtlasMecanumDrive drive;

    private final DcMotor indexer;
    private final DcMotor intake;
    private final DcMotorEx launcher;

    private final ColorSensor launcherSensor;
    private final SwitchableLight launcherSensorLight;
    private final DistanceSensor launcherDistance;

    // ---------------- CONSTANTS ----------------

    private static final double INTAKE_PWR = 1.0;

    private static final double INDEXER_FEED_PWR = 1.0;
    private static final double INDEXER_INTAKE_PWR = 0.6;
    private static final double INDEXER_HOLD_REVERSE_PWR = -0.1;
    private static final double INDEXER_REVERSE_PWR = -1.0;

    private static final double RPM_STEP = 25;
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 1500;

    // Presence detection (tune)
    private static final int SENSOR_ALPHA_THRESHOLD = 100;
    private static final double SENSOR_DIST_MM_THRESHOLD = 45.0;
    private static final double SENSOR_DEBOUNCE_MS = 40;

    // Color classification (simple + stable)
    private static final int COLOR_MIN_SUM = 60;

    // Shooter PIDF
    // Hold PIDF
    private static final double HOLD_P = 20.0, HOLD_I = 0.0, HOLD_D = 3.0, HOLD_F = 18.0;

    // Close PIDF
    private static final double CLOSE_P = 20.0, CLOSE_I = 0.0, CLOSE_D = 3.0, CLOSE_F = 19.0;

    // Far PIDF
    private static final double FAR_P = 20.0, FAR_I = 0.0, FAR_D = 3.0, FAR_F = 19.0;

    // Optional close bias to help recovery and “push through” droop
    private static final double CLOSE_RPM_BIAS = 120.0;

    // ---------------- STATE ----------------
    private final ShooterConfig cfg;

    private RangePreset preset = RangePreset.CLOSE;

    private ShooterMode shooterMode = ShooterMode.OFF;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private IndexerMode indexerMode = IndexerMode.OFF;

    private double manualAdjustRpm = 0.0;
    private double targetRpm = 0.0;

    // Sensor
    private boolean elementRaw = false;
    private boolean elementDebounced = false;
    private boolean elementSeenLatch = false;
    private final ElapsedTime sensorTimer = new ElapsedTime();

    public AtlasFunctions(HardwareMap hw, ShooterConfig shooterCfg) {
        cfg = shooterCfg;

        drive = new AtlasMecanumDrive(hw);

        indexer = hw.get(DcMotor.class, "INDEX_MOTOR");
        intake = hw.get(DcMotor.class, "IMOTOR");
        launcher = hw.get(DcMotorEx.class, "SMOTOR");

        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyHoldPid(); // safe default

        indexer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherSensor = hw.get(ColorSensor.class, "COLOR_SENSOR");
        launcherSensorLight = (launcherSensor instanceof SwitchableLight) ? (SwitchableLight) launcherSensor : null;
        if (launcherSensorLight != null) launcherSensorLight.enableLight(true);

        launcherDistance = (launcherSensor instanceof DistanceSensor) ? (DistanceSensor) launcherSensor : null;

        sensorTimer.reset();
        stopAll();
    }

    // ---------------- DRIVE ----------------
    public void drive(com.qualcomm.robotcore.hardware.Gamepad gp) { drive.drive(gp); }

    public void drive(com.qualcomm.robotcore.hardware.Gamepad gp, double driveScale, double turnScale) {
        drive.drive(gp, driveScale, turnScale);
    }

    public void setDriveMode(AtlasMecanumDrive.DriveMode mode) { drive.setDriveMode(mode); }
    public AtlasMecanumDrive.DriveMode getDriveMode() { return drive.getDriveMode(); }

    public double getHeadingRad() { return drive.getHeadingRadWrapped(); }

    // ================= DRIVE BUILDABLES =================

    public void driveRobotCentric(double forward, double strafe, double turn) {
        drive.driveDirect(forward, strafe, turn);
    }

    public void stopDrive() {
        drive.driveDirect(0, 0, 0);
    }

    public void driveForward(double power) {
        drive.driveDirect(power, 0, 0);
    }

    public void strafe(double power) {
        drive.driveDirect(0, power, 0);
    }

    public void turn(double power) {
        drive.driveDirect(0, 0, power);
    }

    public void turnToAngle(double targetDeg, double power) {
        double currentDeg = Math.toDegrees(getHeadingRad());

        if (currentDeg < targetDeg) {
            drive.driveDirect(0, 0, Math.abs(power));
        } else {
            drive.driveDirect(0, 0, -Math.abs(power));
        }
    }


    // ---------------- PRESETS ----------------
    public void setPreset(RangePreset p) { preset = p; }
    public RangePreset getPreset() { return preset; }

    public double getPresetRpm() {
        return (preset == RangePreset.FAR) ? cfg.farRpm : cfg.closeRpm;
    }

    // ---------------- RPM ADJUSTMENT ----------------
    public double getRpmStep() { return RPM_STEP; }

    public void nudgeManualAdjustRpm(double delta) { manualAdjustRpm += delta; }

    public void resetManualAdjustRpm() { manualAdjustRpm = 0.0; }
    public double getManualAdjustRpm() { return manualAdjustRpm; }

    public double getSelectedTargetRpm() {
        double rpm = getPresetRpm() + manualAdjustRpm;
        return Range.clip(rpm, MIN_RPM, MAX_RPM);
    }

    // ---------------- SHOOTER ----------------
    public void shooterHold() {
        shooterMode = ShooterMode.HOLD;
        targetRpm = cfg.holdRpm;
        applyHoldPid();
        launcher.setVelocity(rpmToTicksPerSec(targetRpm));
    }

    public void shooterTarget(double rpm) {
        shooterMode = ShooterMode.TARGET;

        double desired = Range.clip(rpm, MIN_RPM, MAX_RPM);

        if (preset == RangePreset.CLOSE) {
            applyClosePid();
            desired = Range.clip(desired + CLOSE_RPM_BIAS, MIN_RPM, MAX_RPM);
        } else {
            applyFarPid();
        }

        targetRpm = desired;
        launcher.setVelocity(rpmToTicksPerSec(targetRpm));
    }

    public void shooterOff() {
        shooterMode = ShooterMode.OFF;
        targetRpm = 0.0;
        launcher.setVelocity(0);
    }

    // Keep this method because Atlas calls it, but it is now a no-op (PID is set directly per state/range).
    public void updateShooterPid() { }

    private void applyHoldPid() {
        launcher.setVelocityPIDFCoefficients(HOLD_P, HOLD_I, HOLD_D, HOLD_F);
    }

    private void applyClosePid() {
        launcher.setVelocityPIDFCoefficients(CLOSE_P, CLOSE_I, CLOSE_D, CLOSE_F);
    }

    private void applyFarPid() {
        launcher.setVelocityPIDFCoefficients(FAR_P, FAR_I, FAR_D, FAR_F);
    }

    public ShooterMode getShooterMode() { return shooterMode; }
    public double getTargetRpm() { return targetRpm; }

    public double getMeasuredTicksPerSec() { return launcher.getVelocity(); }

    public double getMeasuredRpm() { return ticksPerSecToRpm(getMeasuredTicksPerSec()); }

    public double getRpmError() { return targetRpm - getMeasuredRpm(); }

    // ---------------- INTAKE ----------------
    public void intakeOn() {
        intakeMode = IntakeMode.IN;
        intake.setPower(INTAKE_PWR);
    }

    public void intakeOff() {
        intakeMode = IntakeMode.OFF;
        intake.setPower(0);
    }

    public void intakeReverse() {
        intakeMode = IntakeMode.OUT;
        intake.setPower(-INTAKE_PWR);
    }

    public IntakeMode getIntakeMode() { return intakeMode; }
    public double getIntakePower() { return intake.getPower(); }

    // ---------------- INDEXER ----------------
    public void indexerFeed() {
        indexerMode = IndexerMode.FEED;
        indexer.setPower(INDEXER_FEED_PWR);
    }

    public void indexerIntake() {
        indexerMode = IndexerMode.FEED;
        indexer.setPower(INDEXER_INTAKE_PWR);
    }

    public void indexerHoldReverse() {
        indexerMode = IndexerMode.HOLD;
        indexer.setPower(INDEXER_HOLD_REVERSE_PWR);
    }

    public void indexerReverse() {
        indexerMode = IndexerMode.REVERSE;
        indexer.setPower(INDEXER_REVERSE_PWR);
    }

    public void indexerOff() {
        indexerMode = IndexerMode.OFF;
        indexer.setPower(0);
    }

    public IndexerMode getIndexerMode() { return indexerMode; }
    public double getIndexerPower() { return indexer.getPower(); }

    // ---------------- SENSOR (presence + color) ----------------
    public void updateElementSensor() {
        boolean now = detectPresentRaw();

        if (now) elementSeenLatch = true;

        if (now != elementRaw) {
            elementRaw = now;
            sensorTimer.reset();
        }

        if (sensorTimer.milliseconds() >= SENSOR_DEBOUNCE_MS) {
            elementDebounced = elementRaw;
        }
    }

    private boolean detectPresentRaw() {
        boolean alphaPresent = launcherSensor.alpha() >= SENSOR_ALPHA_THRESHOLD;

        if (launcherDistance == null) return alphaPresent;

        double mm = launcherDistance.getDistance(DistanceUnit.MM);
        boolean distPresent = Double.isFinite(mm) && mm <= SENSOR_DIST_MM_THRESHOLD;

        return distPresent || alphaPresent;
    }

    public boolean getElementDebounced() { return elementDebounced; }
    public boolean getElementRaw() { return elementRaw; }

    public boolean getElementSeenLatch() { return elementSeenLatch; }
    public void clearElementSeenLatch() { elementSeenLatch = false; }

    public int getSensorAlpha() { return launcherSensor.alpha(); }
    public int getSensorRed() { return launcherSensor.red(); }
    public int getSensorGreen() { return launcherSensor.green(); }
    public int getSensorBlue() { return launcherSensor.blue(); }

    public double getSensorDistanceMm() {
        if (launcherDistance == null) return Double.NaN;
        return launcherDistance.getDistance(DistanceUnit.MM);
    }

    // Stable classification
    public ArtifactType getArtifactType() {
        if (!elementDebounced) return ArtifactType.NONE;

        int r = launcherSensor.red();
        int g = launcherSensor.green();
        int b = launcherSensor.blue();

        int sum = r + g + b;
        if (sum < COLOR_MIN_SUM) return ArtifactType.UNKNOWN;

        int max = Math.max(r, Math.max(g, b));
        int second = (max == r) ? Math.max(g, b) : (max == g ? Math.max(r, b) : Math.max(r, g));

        int margin = (int) (0.06 * sum);

        if (g == max && (g - second) >= margin) return ArtifactType.GREEN;

        if (g != max && (r + b) > (g + margin)) return ArtifactType.PURPLE;

        return ArtifactType.UNKNOWN;
    }

    // ---------------- STOP ALL ----------------
    public void stopAll() {
        shooterOff();
        intakeOff();
        indexerOff();
    }

    // ---------------- CONVERSIONS ----------------
    private double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * cfg.ticksPerRev;
    }

    private double ticksPerSecToRpm(double tps) {
        return (tps / cfg.ticksPerRev) * 60.0;
    }
}
