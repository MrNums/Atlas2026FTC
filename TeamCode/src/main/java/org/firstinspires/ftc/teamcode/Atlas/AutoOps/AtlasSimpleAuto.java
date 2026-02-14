package org.firstinspires.ftc.teamcode.Atlas.AutoOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasFunctions;

@Autonomous(name = "Atlas RED Simple Auto RED", group = "Auto")
public class AtlasSimpleAuto extends OpMode {

    private enum State { DRIVE_OUT, TURN_TO_GOAL, SPIN_UP, SHOOT, PARK, DONE }

    private final ElapsedTime timer = new ElapsedTime();
    private AtlasFunctions fn;
    private State state = State.DRIVE_OUT;
    private int shots;

    private static final double DRIVE_POWER = 0.40;
    private static final long DRIVE_MS = 450;
    private static final double TURN_POWER = -0.45;
    private static final long TURN_MS = 350;

    private static final int TOTAL_SHOTS = 4;
    private static final double READY_ERR_RPM = 170;
    private static final long FEED_MS = 240;
    private static final long GAP_MS = 450;

    @Override
    public void init() {
        fn = new AtlasFunctions(hardwareMap, new AtlasFunctions.ShooterConfig(28, 1500, 2800, 3250));
    }

    @Override
    public void start() {
        fn.stopAll();
        fn.setPreset(AtlasFunctions.RangePreset.FAR);
        state = State.DRIVE_OUT;
        shots = 0;
        timer.reset();
    }

    @Override
    public void loop() {
        switch (state) {
            case DRIVE_OUT:
                fn.driveForward(DRIVE_POWER);
                if (timer.milliseconds() >= DRIVE_MS) {
                    next(State.TURN_TO_GOAL);
                }
                break;

            case TURN_TO_GOAL:
                fn.turn(TURN_POWER);
                if (timer.milliseconds() >= TURN_MS) {
                    next(State.SPIN_UP);
                }
                break;

            case SPIN_UP:
                fn.shooterTarget(fn.getSelectedTargetRpm());
                if (Math.abs(fn.getRpmError()) <= READY_ERR_RPM) {
                    next(State.SHOOT);
                }
                break;

            case SHOOT:
                fn.shooterTarget(fn.getSelectedTargetRpm());
                fn.intakeOn();

                if (shots >= TOTAL_SHOTS) {
                    fn.indexerOff();
                    next(State.PARK);
                    break;
                }

                if (timer.milliseconds() < FEED_MS) {
                    fn.indexerFeed();
                } else if (timer.milliseconds() < FEED_MS + GAP_MS) {
                    fn.indexerOff();
                } else {
                    shots++;
                    timer.reset();
                }
                break;

            case PARK:
                fn.driveForward(0.60);
                if (timer.milliseconds() >= DRIVE_MS) {
                    next(State.DONE);
                }
                break;

            case DONE:
                fn.stopAll();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Shots", shots);
        telemetry.addData("RPM err", "%.1f", fn.getRpmError());
        telemetry.update();
    }

    private void next(State next) {
        fn.stopDrive();
        timer.reset();
        state = next;
    }
}
