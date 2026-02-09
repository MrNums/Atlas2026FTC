package org.firstinspires.ftc.teamcode.Atlas.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Atlas.Utils.AtlasFunctions;

@TeleOp(name = "Atlas Go", group = "Iterative Opmode")
public class Atlas extends OpMode {

    private AtlasFunctions fn;

    @Override
    public void init() {
        fn = new AtlasFunctions(hardwareMap);
        telemetry.addLine("Init OK");
        telemetry.update();
    }

    @Override
    public void start() {
        fn.stopAll();
    }

    @Override
    public void loop() {

        fn.updateRPMAdjust(gamepad1);
        fn.updateShooter(gamepad1);
        fn.updateIntake(gamepad1);
        fn.updateIndexer(gamepad1);

        fn.drive(gamepad1);

        telemetry.addData("DriveMode", fn.getDriveMode());
        telemetry.addData("Heading(rad)", "%.3f", fn.getHeadingRad());

        telemetry.addData("Adjustable RPM", fn.getLauncherRPM());
        telemetry.addData("Long Range RPM", fn.getLongRangeRPM());
        telemetry.addData("Shooter Velocity", fn.getShooterVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        fn.stopAll();
    }
}