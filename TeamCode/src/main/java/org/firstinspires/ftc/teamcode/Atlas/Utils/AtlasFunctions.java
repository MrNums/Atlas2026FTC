package org.firstinspires.ftc.teamcode.Atlas.Utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    public enum IntakeMode { OFF, IN }
    public enum IndexerMode { OFF, FEED, HOLD_REVERSE }

    // ---------------- HARDWARE ----------------

    private final AtlasMecanumDrive drive;

    private final DcMotor indexer;
    private final DcMotor intake;
    private final DcMotorEx launcher;

    private final ColorSensor launcherSensor;
    private final SwitchableLight launcherSensorLight;

    // ---------------- CONSTANTS ----------------

    private static final double INTAKE_PWR = 1.0;
    private static final double INDEXER_FEED_PWR = 1.0;
    private static final double INDEXER_HOLD_REVERSE_PWR = -0.10;

    private static final double RPM_STEP = 25;
    private static final double MAX_RPM = 3000;
    private static final double MIN_RPM = 1000;

    private static final int SENSOR_ALPHA_THRESHOLD = 80; // tune
    private static final double SENSOR_DEBOUNCE_MS = 60;

    // ---------------- STATE ----------------

    private final ShooterConfig cfg;

    private RangePreset preset = RangePreset.CLOSE;

    private ShooterMode shooterMode = ShooterMode.OFF;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private IndexerMode indexerMode = IndexerMode.OFF;

    private double manualAdjustRpm = 0.0;
    private double targetRpm = 0.0;

    private boolean elementRaw = false;
    private boolean elementDebounced = false;
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

        indexer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherSensor = hw.get(ColorSensor.class, "COLOR_SENSOR");
        launcherSensorLight = (launcherSensor instanceof SwitchableLight) ? (SwitchableLight) launcherSensor : null;
        if (launcherSensorLight != null) launcherSensorLight.enableLight(true);

        sensorTimer.reset();
        stopAll();
    }

    // ---------------- DRIVE ----------------

    public void drive(com.qualcomm.robotcore.hardware.Gamepad gp) {
        drive.drive(gp);
    }

    public void drive(com.qualcomm.robotcore.hardware.Gamepad gp, double driveScale, double turnScale) {
        drive.drive(gp, driveScale, turnScale);
    }

    public void setDriveMode(AtlasMecanumDrive.DriveMode mode) { drive.setDriveMode(mode); }
    public AtlasMecanumDrive.DriveMode getDriveMode() { return drive.getDriveMode(); }
    public double getHeadingRad() { return drive.getHeadingRad(); }

    // ---------------- PRESETS ----------------

    public void setPreset(RangePreset p) { preset = p; }
    public RangePreset getPreset() { return preset; }

    public double getPresetRpm() {
        return (preset == RangePreset.FAR) ? cfg.farRpm : cfg.closeRpm;
    }

    // ---------------- RPM ADJUSTMENT ----------------

    public double getRpmStep() { return RPM_STEP; }

    public void nudgeManualAdjustRpm(double delta) {
        manualAdjustRpm += delta;
    }

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
        launcher.setVelocity(rpmToTicksPerSec(targetRpm));
    }

    public void shooterTarget(double rpm) {
        shooterMode = ShooterMode.TARGET;
        targetRpm = Range.clip(rpm, MIN_RPM, MAX_RPM);
        launcher.setVelocity(rpmToTicksPerSec(targetRpm));
    }

    public void shooterOff() {
        shooterMode = ShooterMode.OFF;
        targetRpm = 0.0;
        launcher.setVelocity(0);
    }

    public ShooterMode getShooterMode() { return shooterMode; }
    public double getTargetRpm() { return targetRpm; }

    public double getMeasuredTicksPerSec() { return launcher.getVelocity(); }

    public double getMeasuredRpm() {
        return ticksPerSecToRpm(getMeasuredTicksPerSec());
    }

    public double getRpmError() {
        return targetRpm - getMeasuredRpm();
    }

    // ---------------- INTAKE ----------------

    public void intakeOn() {
        intakeMode = IntakeMode.IN;
        intake.setPower(INTAKE_PWR);
    }

    public void intakeOff() {
        intakeMode = IntakeMode.OFF;
        intake.setPower(0);
    }

    public IntakeMode getIntakeMode() { return intakeMode; }
    public double getIntakePower() { return intake.getPower(); }

    // ---------------- INDEXER ----------------

    public void indexerFeed() {
        indexerMode = IndexerMode.FEED;
        indexer.setPower(INDEXER_FEED_PWR);
    }

    public void indexerHoldReverse() {
        indexerMode = IndexerMode.HOLD_REVERSE;
        indexer.setPower(INDEXER_HOLD_REVERSE_PWR);
    }

    public void indexerOff() {
        indexerMode = IndexerMode.OFF;
        indexer.setPower(0);
    }

    public IndexerMode getIndexerMode() { return indexerMode; }
    public double getIndexerPower() { return indexer.getPower(); }

    // ---------------- SENSOR ----------------

    public void updateElementSensor() {
        boolean now = launcherSensor.alpha() >= SENSOR_ALPHA_THRESHOLD;

        if (now != elementRaw) {
            elementRaw = now;
            sensorTimer.reset();
        }

        if (sensorTimer.milliseconds() >= SENSOR_DEBOUNCE_MS) {
            elementDebounced = elementRaw;
        }
    }

    public int getSensorAlpha() { return launcherSensor.alpha(); }
    public boolean getElementRaw() { return elementRaw; }
    public boolean getElementDebounced() { return elementDebounced; }

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