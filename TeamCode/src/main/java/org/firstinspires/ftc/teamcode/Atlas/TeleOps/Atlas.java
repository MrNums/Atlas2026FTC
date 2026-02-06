package org.firstinspires.ftc.teamcode.Atlas.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Atlas Go")
public class Atlas extends LinearOpMode {

    private NewMecanumDrivetrain drive;
    private DcMotor indexer;
    private DcMotor intake;
    private DcMotorEx launcher;
    private DcMotorEx launcherv2;
    private IMU imu_IMU;

    private double launcherRPM = 1260;
    private double initialRPM = 1260;
    private double rpmStep = 10;
    private double maxRPM = 3000;
    private double minRPM = 1000;

    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevRight = false;

    private final double longRangeRPM = 1575;

    @Override
    public void runOpMode() {

        drive = new NewMecanumDrivetrain(hardwareMap);
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherv2 = hardwareMap.get(DcMotorEx.class, "launcherv2");
        imu_IMU = hardwareMap.get(IMU.class, "imu");

        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcherv2.setDirection(DcMotor.Direction.FORWARD);

        launcher.setVelocity(0);
        launcherv2.setVelocity(0);

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ---------------- RPM ADJUSTMENT ----------------
            if (gamepad1.dpad_up && !prevUp) {
                launcherRPM += rpmStep;
                launcherRPM = Range.clip(launcherRPM, minRPM, maxRPM);
                gamepad1.rumble(200);
            }
            prevUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !prevDown) {
                launcherRPM -= rpmStep;
                launcherRPM = Range.clip(launcherRPM, minRPM, maxRPM);
                gamepad1.rumble(200);
            }
            prevDown = gamepad1.dpad_down;

            if (gamepad1.dpad_right && !prevRight) {
                launcherRPM = initialRPM;
                gamepad1.rumbleBlips(2);
            }
            prevRight = gamepad1.dpad_right;

            // ---------------- SHOOTER ----------------
            if (gamepad1.right_bumper) {
                launcher.setVelocity(longRangeRPM);
                launcherv2.setVelocity(longRangeRPM);
            } else if (gamepad1.right_trigger > 0.5) {
                launcher.setVelocity(launcherRPM);
                launcherv2.setVelocity(launcherRPM);
            } else {
                launcher.setVelocity(0);
                launcherv2.setVelocity(0);
            }

            // ---------------- INTAKE + INDEXER ----------------
            if (gamepad1.left_trigger > 0.5){
                intake.setDirection(DcMotor.Direction.REVERSE);
                intake.setPower(1);
            }else{
                intake.setPower(0);
            }

            if (gamepad1.left_bumper){
                indexer.setDirection(DcMotor.Direction.FORWARD );
                indexer.setPower(.5);
            }else{
                indexer.setPower(0);
            }

            // ---------------- UNJAM ----------------
            //if (gamepad1.dpad_left) {
            //intake.setPower(1);
            //indexer.setPower(1);
            //intake.setDirection(DcMotor.Direction.FORWARD);
            //indexer.setDirection(DcMotor.Direction.REVERSE);
            //} else {
            //intake.setDirection(DcMotor.Direction.REVERSE);
            //indexer.setDirection(DcMotor.Direction.FORWARD);
            //intake.setPower(0);
            //indexer.setPower(0);
            //}

            // ---------------- RESET YAW ----------------
            if (gamepad1.left_stick_button) {
                imu_IMU.resetYaw();
            }

            // ---------------- DRIVE ----------------
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;
            drive.driveRobotCentric(x, y, rot);

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Adjustable RPM", launcherRPM);
            telemetry.addData("Long Range RPM", longRangeRPM);
            telemetry.addData("Shooter Velocity", launcher.getVelocity());
            telemetry.update();
        }
    }
}