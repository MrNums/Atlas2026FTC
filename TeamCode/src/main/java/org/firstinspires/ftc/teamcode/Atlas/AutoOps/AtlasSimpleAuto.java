package org.firstinspires.ftc.teamcode.Atlas.AutoOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasFunctions;

@Autonomous(name = "Atlas RED Simple Auto RED", group = "Auto")
public class AtlasSimpleAuto extends OpMode {

    private AtlasFunctions fn;
    private ElapsedTime t = new ElapsedTime();

    private enum State {
        FORWARD,
        TURN,
        SPINUP,
        SHOOT,
        TURN2,
        FORWARD2,
        DONE
    }

    private State state = State.FORWARD;

    // DRIVE
    private static final double FORWARD_PWR = 0.4;
    private static final long   FORWARD_MS  = 400;

    private static final double TURN_PWR = 0.5;
    private static final double TURN_DEG = -15.0;
    private static final double TURN_DEG2 = -1.0;

    // SHOOTING
    private static final double READY_ERR_RPM = 150;

    private static final int SHOTS = 4;
    private static final long FEED_MS = 235;
    private static final long GAP_MS  = 600;

    private int shotCount = 0;

    @Override
    public void init() {

        AtlasFunctions.ShooterConfig cfg =
                new AtlasFunctions.ShooterConfig(
                        28,
                        1500,
                        2800,
                        3250
                );

        fn = new AtlasFunctions(hardwareMap, cfg);
    }

    @Override
    public void start() {
        t.reset();
        shotCount = 0;
        state = State.FORWARD;
        fn.stopAll();
    }

    @Override
    public void loop() {

        switch (state) {

            case FORWARD:

                fn.driveForward(FORWARD_PWR);

                if (t.milliseconds() >= FORWARD_MS) {
                    fn.stopDrive();
                    t.reset();
                    state = State.TURN;
                }

                break;

            case TURN:

                double headingDeg = Math.toDegrees(fn.getHeadingRad());

                if (headingDeg > TURN_DEG) {
                    fn.turn(TURN_PWR);
                } else {
                    fn.stopDrive();
                    fn.setPreset(AtlasFunctions.RangePreset.FAR);
                    t.reset();
                    state = State.SPINUP;
                }

                break;

            case SPINUP:

                fn.shooterTarget(fn.getSelectedTargetRpm());
                fn.updateShooterPid();

                if (Math.abs(fn.getRpmError()) <= READY_ERR_RPM) {
                    t.reset();
                    state = State.SHOOT;
                }

                break;

            case SHOOT:

                fn.shooterTarget(fn.getSelectedTargetRpm());
                fn.updateShooterPid();
                fn.intakeOn();

                if (shotCount >= SHOTS) {
                    fn.indexerOff();
                    t.reset();
                    state = State.FORWARD2;
                    break;
                }

                double cycleTime = t.milliseconds();

                if (cycleTime < FEED_MS) {
                    fn.indexerFeed();
                }
                else if (cycleTime < FEED_MS + GAP_MS) {
                    fn.indexerOff();
                }
                else {
                    shotCount++;
                    t.reset();
                }

                break;

            case FORWARD2:

                fn.driveForward(FORWARD_PWR + 0.4);

                if (t.milliseconds() >= FORWARD_MS) {
                    fn.stopDrive();
                    state = State.DONE;
                }

                break;

            case DONE:
                fn.stopAll();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(fn.getHeadingRad()));
        telemetry.addData("Shots", shotCount);
        telemetry.update();
    }
}