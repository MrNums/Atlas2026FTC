package org.firstinspires.ftc.teamcode.Atlas.Utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
            if (ticksPerRev <= 0) {
                throw new IllegalArgumentException("ticksPerRev must be > 0");
            }
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

    private static final double INTAKE_PWR = 1.0;
    private static final double INDEXER_FEED_PWR = 1.0;
    private static final double INDEXER_INTAKE_PWR = 0.65;
    private static final double INDEXER_HOLD_REVERSE_PWR = -0.1;
    private static final double INDEXER_REVERSE_PWR = -1.0;
    private static final double RPM_STEP = 25;

    private static final double MIN_RPM = 1500;
    private static final double MAX_RPM = 6000;

    private static final int SENSOR_ALPHA_THRESHOLD = 100;
    private static final double SENSOR_DIST_MM_THRESHOLD = 45.0;
    private static final double SENSOR_DEBOUNCE_MS = 40;

    private final ShooterConfig cfg;
    private final AtlasMecanumDrive drive;

    private final DcMotor indexer;
    private final DcMotor intake;
    private final DcMotorEx launcher;
    private final ColorSensor launcherSensor;
    private final DistanceSensor launcherDistance;

    private final ElapsedTime sensorDebounceTimer = new ElapsedTime();

    private RangePreset preset = RangePreset.CLOSE;
    private ShooterMode shooterMode = ShooterMode.OFF;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private IndexerMode indexerMode = IndexerMode.OFF;

    private boolean elementRaw;
    private boolean elementDebounced;
    private boolean elementSeenLatch;

    private double targetRpm;
    private double manualAdjustRpm;

    public AtlasFunctions(HardwareMap hw, ShooterConfig shooterCfg) {
        cfg = shooterCfg;
        drive = new AtlasMecanumDrive(hw);

        indexer = hw.get(DcMotor.class, "INDEX_MOTOR");
        intake = hw.get(DcMotor.class, "IMOTOR");
        launcher = hw.get(DcMotorEx.class, "SMOTOR");

        indexer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherSensor = hw.get(ColorSensor.class, "COLOR_SENSOR");
        if (launcherSensor instanceof SwitchableLight) {
            ((SwitchableLight) launcherSensor).enableLight(true);
        }
        launcherDistance = launcherSensor instanceof DistanceSensor ? (DistanceSensor) launcherSensor : null;

        stopAll();
        sensorDebounceTimer.reset();
    }

    public void drive(Gamepad gp) { drive.drive(gp); }
    public void drive(Gamepad gp, double driveScale, double turnScale) { drive.drive(gp, driveScale, turnScale); }
    public void setDriveMode(AtlasMecanumDrive.DriveMode mode) { drive.setDriveMode(mode); }
    public AtlasMecanumDrive.DriveMode getDriveMode() { return drive.getDriveMode(); }
    public double getHeadingRad() { return drive.getHeadingRadWrapped(); }

    public void driveRobotCentric(double forward, double strafe, double turn) { drive.driveDirect(forward, strafe, turn); }
    public void driveForward(double power) { drive.driveDirect(power, 0, 0); }
    public void strafe(double power) { drive.driveDirect(0, power, 0); }
    public void turn(double power) { drive.driveDirect(0, 0, power); }
    public void stopDrive() { drive.driveDirect(0, 0, 0); }

    public void setPreset(RangePreset p) {
        if (p != null) preset = p;
    }
    public RangePreset getPreset() { return preset; }
    public double getClosePresetRpm() { return cfg.closeRpm; }
    public double getFarPresetRpm() { return cfg.farRpm; }
    public double getRpmStep() { return RPM_STEP; }

    public void nudgeManualAdjustRpm(double delta) { manualAdjustRpm += delta; }
    public void resetManualAdjustRpm() { manualAdjustRpm = 0; }
    public double getManualAdjustRpm() { return manualAdjustRpm; }

    public double getPresetRpm() {
        return preset == RangePreset.FAR ? cfg.farRpm : cfg.closeRpm;
    }

    public double getSelectedTargetRpm() {
        return Range.clip(getPresetRpm() + manualAdjustRpm, MIN_RPM, MAX_RPM);
    }

    public void shooterHold() {
        shooterMode = ShooterMode.HOLD;
        setShooterRpm(cfg.holdRpm);
    }

    public void shooterTarget(double rpm) {
        shooterMode = ShooterMode.TARGET;
        setShooterRpm(rpm);
    }

    public void shooterOff() {
        shooterMode = ShooterMode.OFF;
        targetRpm = 0;
        launcher.setVelocity(0);
    }

    public void updateShooterPid() {
        // intentionally simple; we use motor's built-in velocity control only
    }

    public ShooterMode getShooterMode() { return shooterMode; }
    public double getTargetRpm() { return targetRpm; }
    public double getMeasuredTicksPerSec() { return launcher.getVelocity(); }
    public double getMeasuredRpm() { return ticksPerSecToRpm(getMeasuredTicksPerSec()); }
    public double getRpmError() { return targetRpm - getMeasuredRpm(); }

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

    public void updateElementSensor() {
        boolean now = detectPresentRaw();
        if (now) elementSeenLatch = true;

        if (now != elementRaw) {
            elementRaw = now;
            sensorDebounceTimer.reset();
        }

        if (sensorDebounceTimer.milliseconds() > SENSOR_DEBOUNCE_MS) {
            elementDebounced = elementRaw;
        }
    }

    private boolean detectPresentRaw() {
        boolean alphaPresent = launcherSensor.alpha() >= SENSOR_ALPHA_THRESHOLD;
        if (launcherDistance == null) return alphaPresent;

        double distMm = launcherDistance.getDistance(DistanceUnit.MM);
        boolean distPresent = Double.isFinite(distMm) && distMm <= SENSOR_DIST_MM_THRESHOLD;
        return alphaPresent || distPresent;
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
        return launcherDistance == null ? Double.NaN : launcherDistance.getDistance(DistanceUnit.MM);
    }

    public ArtifactType getArtifactType() {
        if (!elementDebounced) return ArtifactType.NONE;

        int r = launcherSensor.red();
        int g = launcherSensor.green();
        int b = launcherSensor.blue();

        if (g > r && g > b) return ArtifactType.GREEN;
        if (r + b > g) return ArtifactType.PURPLE;
        return ArtifactType.UNKNOWN;
    }

    public void stopAll() {
        shooterOff();
        intakeOff();
        indexerOff();
        stopDrive();
    }

    private void setShooterRpm(double rpm) {
        targetRpm = Range.clip(rpm, MIN_RPM, MAX_RPM);
        launcher.setVelocity(rpmToTicksPerSec(targetRpm));
    }

    private double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * cfg.ticksPerRev;
    }

    private double ticksPerSecToRpm(double ticksPerSecond) {
        return (ticksPerSecond / cfg.ticksPerRev) * 60.0;
    }
}
