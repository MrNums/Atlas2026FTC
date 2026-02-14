package org.firstinspires.ftc.teamcode.Atlas.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasFunctions;
import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasMecanumDrive;

@TeleOp(name = "Atlas Simple TeleOp", group = "Atlas")
public class Atlas extends OpMode {

    private AtlasFunctions fn;

    private boolean lastA;
    private boolean lastB;
    private boolean lastY;
    private boolean lastUp;
    private boolean lastDown;
    private boolean lastRight;

    private static final double READY_ERR_RPM = 220;

    @Override
    public void init() {
        fn = new AtlasFunctions(hardwareMap, new AtlasFunctions.ShooterConfig(28, 1500, 2800, 3250));
        telemetry.addLine("Atlas simple teleop ready");
        telemetry.addData("Close preset RPM", "%.0f", fn.getClosePresetRpm());
        telemetry.addData("Far preset RPM", "%.0f", fn.getFarPresetRpm());
        telemetry.addLine("A = CLOSE preset, B = FAR preset");
        telemetry.update();
    }

    @Override
    public void start() {
        fn.stopAll();
    }

    @Override
    public void loop() {
        handleButtons();

        boolean aiming = gamepad1.left_bumper;
        boolean fire = gamepad1.right_trigger > 0.5;
        boolean intake = gamepad1.left_trigger > 0.5;
        boolean reverse = gamepad1.right_bumper;

        fn.drive(gamepad1, 1.0, aiming ? 0.45 : 1.0);

        if (aiming) {
            fn.shooterTarget(fn.getSelectedTargetRpm());
        } else {
            fn.shooterHold();
        }

        if (reverse) {
            fn.intakeReverse();
            fn.indexerReverse();
        } else if (fire && aiming && Math.abs(fn.getRpmError()) <= READY_ERR_RPM) {
            fn.intakeOn();
            fn.indexerFeed();
        } else if (intake) {
            fn.intakeOn();
            fn.indexerIntake();
        } else {
            fn.intakeOff();
            fn.indexerOff();
        }

        telemetry.addData("Preset", fn.getPreset());
        telemetry.addData("Preset RPM", "%.0f", fn.getPresetRpm());
        telemetry.addData("Manual trim", "%.0f", fn.getManualAdjustRpm());
        telemetry.addData("Selected target", "%.0f", fn.getSelectedTargetRpm());
        telemetry.addData("Drive", fn.getDriveMode());
        telemetry.addData("Target rpm", "%.0f", fn.getTargetRpm());
        telemetry.addData("Measured rpm", "%.0f", fn.getMeasuredRpm());
        telemetry.addData("Ready", Math.abs(fn.getRpmError()) <= READY_ERR_RPM);
        telemetry.update();
    }

    @Override
    public void stop() {
        fn.stopAll();
    }

    private void handleButtons() {
        boolean aNow = gamepad1.a;
        boolean bNow = gamepad1.b;
        boolean yNow = gamepad1.y;
        boolean upNow = gamepad1.dpad_up;
        boolean downNow = gamepad1.dpad_down;
        boolean rightNow = gamepad1.dpad_right;

        if (aNow && !lastA) fn.setPreset(AtlasFunctions.RangePreset.CLOSE);
        if (bNow && !lastB) fn.setPreset(AtlasFunctions.RangePreset.FAR);

        if (upNow && !lastUp) fn.nudgeManualAdjustRpm(fn.getRpmStep());
        if (downNow && !lastDown) fn.nudgeManualAdjustRpm(-fn.getRpmStep());
        if (rightNow && !lastRight) fn.resetManualAdjustRpm();

        if (yNow && !lastY) {
            AtlasMecanumDrive.DriveMode next =
                    fn.getDriveMode() == AtlasMecanumDrive.DriveMode.FIELD_CENTRIC
                            ? AtlasMecanumDrive.DriveMode.ROBOT_CENTRIC
                            : AtlasMecanumDrive.DriveMode.FIELD_CENTRIC;
            fn.setDriveMode(next);
        }

        lastA = aNow;
        lastB = bNow;
        lastY = yNow;
        lastUp = upNow;
        lastDown = downNow;
        lastRight = rightNow;
    }
}
