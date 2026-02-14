package org.firstinspires.ftc.teamcode.Atlas.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasFunctions;
import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasMecanumDrive;

@TeleOp(name = "Atlas Go", group = "Iterative Opmode")
public class Atlas extends OpMode {

    private final ElapsedTime time = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private AtlasFunctions fn;

    private enum SpeedMode { NORMAL, AIMING }
    private SpeedMode speedMode = SpeedMode.NORMAL;

    private enum FireState { IDLE, READY, FEEDING, PAUSE, RECOVER, FAULT }
    private FireState fireState = FireState.IDLE;

    private boolean lastA = false;
    private boolean lastB = false;

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastRight = false;

    private boolean lastY = false;
    private boolean lastElem = false;

    private static final boolean USE_INDEXER_HOLD_REVERSE_WHEN_STAGED = true;

    private static final double DRIVE_SCALE = 1.0;
    private static final double TURN_NORMAL = 1.0;
    private static final double TURN_AIMING = 0.45;

    private static final double RPM_READY_PCT = 0.95;
    private static final double RPM_RECOVER_PCT = 0.96;

    private static final double PAUSE_MS = 90;
    private static final double MAX_FEED_MS = 1500;

    private static final double FEED_PULSE_MS = 350;
    private static final double SHOT_GAP_MS   = 650;
    private static final double RPM_FEED_PCT  = 0.90;

    private static final double READY_ERR_ALLOW_CLOSE_RPM = 400;
    private static final double READY_ERR_ALLOW_FAR_RPM   = 200;

    @Override
    public void init() {

        AtlasFunctions.ShooterConfig cfg = new AtlasFunctions.ShooterConfig(
                28,
                1500,
                2800,
                3250
        );

        fn = new AtlasFunctions(hardwareMap, cfg);

        telemetry.addLine("Init OK");
        telemetry.update();
    }

    @Override
    public void start() {
        time.reset();
        stateTimer.reset();
        shotTimer.reset();
        fireState = FireState.IDLE;
        fn.stopAll();
    }

    @Override
    public void loop() {

        // ---------------- INPUTS ----------------

        boolean aimHeld = gamepad1.left_bumper;          // aiming/spooling
        boolean fireHeld = gamepad1.right_trigger > 0.5; // normal fire logic
        boolean intakeHeld = gamepad1.left_trigger > 0.5;
        boolean reverseHeld = gamepad1.right_bumper;

        boolean bypassFireHeld = gamepad1.x;

        boolean aNow = gamepad1.a;
        boolean bNow = gamepad1.b;

        boolean upNow = gamepad1.dpad_up;
        boolean downNow = gamepad1.dpad_down;
        boolean rightNow = gamepad1.dpad_right;

        // ---------------- PRESETS ----------------

        if (aNow && !lastA) fn.setPreset(AtlasFunctions.RangePreset.CLOSE);
        if (bNow && !lastB) fn.setPreset(AtlasFunctions.RangePreset.FAR);
        lastA = aNow;
        lastB = bNow;

        // ---------------- RPM ADJUSTMENT ----------------

        if (upNow && !lastUp) fn.nudgeManualAdjustRpm(+fn.getRpmStep());
        if (downNow && !lastDown) fn.nudgeManualAdjustRpm(-fn.getRpmStep());
        if (rightNow && !lastRight) fn.resetManualAdjustRpm();

        lastUp = upNow;
        lastDown = downNow;
        lastRight = rightNow;

        // ---------------- DRIVE MODE TOGGLE ----------------

        boolean yNow = gamepad1.y;
        if (yNow && !lastY) {
            AtlasMecanumDrive.DriveMode next =
                    (fn.getDriveMode() == AtlasMecanumDrive.DriveMode.FIELD_CENTRIC)
                            ? AtlasMecanumDrive.DriveMode.ROBOT_CENTRIC
                            : AtlasMecanumDrive.DriveMode.FIELD_CENTRIC;
            fn.setDriveMode(next);
        }
        lastY = yNow;

        // ---------------- SPEED MODE ----------------

        speedMode = aimHeld ? SpeedMode.AIMING : SpeedMode.NORMAL;
        double turnScale = (speedMode == SpeedMode.AIMING) ? TURN_AIMING : TURN_NORMAL;
        fn.drive(gamepad1, DRIVE_SCALE, turnScale);

        // ---------------- SENSOR ----------------

        fn.updateElementSensor();
        boolean elem = fn.getElementDebounced();
        boolean elemRaw = fn.getElementRaw();
        boolean elemSeen = fn.getElementSeenLatch();

        boolean elemRising = elem && !lastElem;
        lastElem = elem;

        // ---------------- SHOOTER ----------------

        if (aimHeld) fn.shooterTarget(fn.getSelectedTargetRpm());
        else fn.shooterHold();

        fn.updateShooterPid();

        // ---------------- BYPASS FIRE (aiming only) ----------------

        if (aimHeld && bypassFireHeld) {
            fn.intakeOn();
            fn.indexerFeed();
            fireState = FireState.IDLE;

            telemetry.addLine("=== BYPASS FIRE (AIMING) ACTIVE ===");
            telemetry.addData("Target", "%.1f", fn.getTargetRpm());
            telemetry.addData("Meas", "%.1f", fn.getMeasuredRpm());
            telemetry.addData("Err", "%.1f", fn.getRpmError());
            telemetry.update();
            return;
        }

        // ---------------- INTAKE + INDEXER ----------------

        boolean fireActive = aimHeld && fireHeld;

        if (!fireActive) {

            fireState = FireState.IDLE;

            if (intakeHeld) {
                fn.intakeOn();

                boolean stagedOrSeen = elem || elemRaw || elemSeen;

                if (!stagedOrSeen) {
                    fn.indexerIntake();
                } else {
                    if (USE_INDEXER_HOLD_REVERSE_WHEN_STAGED) fn.indexerHoldReverse();
                    else fn.indexerOff();
                }

                if (reverseHeld) {
                    fn.indexerReverse();
                    fn.intakeReverse();
                }

            } else {
                fn.intakeOff();
                fn.indexerOff();
                fn.clearElementSeenLatch();
            }

        } else {

            double target = fn.getTargetRpm();
            double meas = fn.getMeasuredRpm();

            double err = target - meas; // positive means low

            double errAllow = (fn.getPreset() == AtlasFunctions.RangePreset.CLOSE)
                    ? READY_ERR_ALLOW_CLOSE_RPM
                    : READY_ERR_ALLOW_FAR_RPM;

            switch (fireState) {

                case IDLE:
                    fireState = FireState.READY;
                    stateTimer.reset();
                    break;

                case READY:
                    fn.intakeOn();
                    fn.indexerOff();

                    boolean cooldownOk = shotTimer.milliseconds() >= SHOT_GAP_MS;

                    boolean atSpeedAbs = err <= errAllow;
                    boolean atSpeedPct = meas >= target * RPM_FEED_PCT;

                    if ((atSpeedAbs || atSpeedPct) && cooldownOk) {
                        fn.clearElementSeenLatch();
                        fireState = FireState.FEEDING;
                        stateTimer.reset();
                    }
                    break;

                case FEEDING:
                    fn.intakeOn();

                    if (stateTimer.milliseconds() <= FEED_PULSE_MS) {
                        fn.indexerFeed();
                    } else {
                        fn.indexerOff();
                        shotTimer.reset();
                        fireState = FireState.PAUSE;
                        stateTimer.reset();
                    }

                    if (stateTimer.milliseconds() > MAX_FEED_MS) {
                        fireState = FireState.FAULT;
                        stateTimer.reset();
                    }
                    break;

                case PAUSE:
                    fn.intakeOn();
                    fn.indexerOff();
                    if (stateTimer.milliseconds() >= PAUSE_MS) {
                        fireState = FireState.RECOVER;
                        stateTimer.reset();
                    }
                    break;

                case RECOVER:
                    fn.intakeOn();
                    fn.indexerOff();

                    if (meas >= target * RPM_RECOVER_PCT) {
                        if (aimHeld && fireHeld) {
                            fireState = FireState.READY;
                            stateTimer.reset();
                        } else {
                            fireState = FireState.IDLE;
                        }
                    }
                    break;

                case FAULT:
                    fn.intakeOn();
                    fn.indexerOff();
                    if (meas >= target * RPM_READY_PCT) {
                        fireState = FireState.READY;
                        stateTimer.reset();
                    }
                    break;
            }
        }

        // ---------------- TELEMETRY ----------------

        telemetry.addLine("=== Drive ===");
        telemetry.addData("Time", time.toString());
        telemetry.addData("SpeedMode", speedMode);
        telemetry.addData("DriveMode", fn.getDriveMode());
        telemetry.addData("Heading(rad)", "%.3f", fn.getHeadingRad());

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Preset", fn.getPreset());
        telemetry.addData("Adj", "%.0f", fn.getManualAdjustRpm());
        telemetry.addData("Target", "%.1f", fn.getTargetRpm());
        telemetry.addData("Meas", "%.1f", fn.getMeasuredRpm());
        telemetry.addData("Err", "%.1f", fn.getRpmError());
        telemetry.addData("Mode", fn.getShooterMode());
        telemetry.addData("FireState", fireState);

        telemetry.addLine("=== Element ===");
        telemetry.addData("Alpha", fn.getSensorAlpha());
        telemetry.addData("Dist(mm)", "%.1f", fn.getSensorDistanceMm());
        telemetry.addData("Present", fn.getElementDebounced());
        telemetry.addData("Raw", fn.getElementRaw());
        telemetry.addData("SeenLatch", fn.getElementSeenLatch());
        telemetry.addData("RGB", "%d %d %d", fn.getSensorRed(), fn.getSensorGreen(), fn.getSensorBlue());
        telemetry.addData("Type", fn.getArtifactType());

        telemetry.update();
    }

    @Override
    public void stop() {
        fn.stopAll();
    }
}