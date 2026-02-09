package org.firstinspires.ftc.teamcode.Atlas.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class AtlasFunctions {

    private final AtlasMecanumDrive drive;

    private final DcMotor indexer;
    private final DcMotor intake;
    private final DcMotorEx launcher;
    private double launcherRPM = 1260;
    private final double initialRPM = 1260;
    private final double rpmStep = 10;
    private final double maxRPM = 3000;
    private final double minRPM = 1000;

    private final double longRangeRPM = 1575;

    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevRight = false;

    public AtlasFunctions(HardwareMap hardwareMap) {

        drive = new AtlasMecanumDrive(hardwareMap);

        indexer = hardwareMap.get(DcMotor.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        indexer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        launcher.setDirection(DcMotor.Direction.REVERSE);

        stopAll();
    }

    // ---------------- DRIVE ----------------

    public void drive(Gamepad gp) {
        drive.drive(gp);
    }

    public void setDriveMode(AtlasMecanumDrive.DriveMode mode) {
        drive.setDriveMode(mode);
    }

    public AtlasMecanumDrive.DriveMode getDriveMode() {
        return drive.getDriveMode();
    }

    public double getHeadingRad() {
        return drive.getHeadingRad();
    }

    // ---------------- RPM ADJUSTMENT ----------------

    public void updateRPMAdjust(Gamepad gp) {

        if (gp.dpad_up && !prevUp) {
            launcherRPM += rpmStep;
            launcherRPM = Range.clip(launcherRPM, minRPM, maxRPM);
            gp.rumble(200);
        }
        prevUp = gp.dpad_up;

        if (gp.dpad_down && !prevDown) {
            launcherRPM -= rpmStep;
            launcherRPM = Range.clip(launcherRPM, minRPM, maxRPM);
            gp.rumble(200);
        }
        prevDown = gp.dpad_down;

        if (gp.dpad_right && !prevRight) {
            launcherRPM = initialRPM;
            gp.rumbleBlips(2);
        }
        prevRight = gp.dpad_right;
    }

    // ---------------- SHOOTER ----------------

    public void updateShooter(Gamepad gp) {

        if (gp.right_bumper) {
            setShooterVelocity(longRangeRPM);
        } else if (gp.right_trigger > 0.5) {
            setShooterVelocity(launcherRPM);
        } else {
            setShooterVelocity(0);
        }
    }

    private void setShooterVelocity(double v) {
        launcher.setVelocity(v);
    }

    public void stopShooter() {
        setShooterVelocity(0);
    }

    // ---------------- INTAKE ----------------

    public void updateIntake(Gamepad gp) {

        if (gp.left_trigger > 0.5) {
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    // ---------------- INDEXER ----------------

    public void updateIndexer(Gamepad gp) {

        if (gp.left_bumper) {
            indexer.setDirection(DcMotor.Direction.FORWARD);
            indexer.setPower(0.5);
        } else {
            indexer.setPower(0);
        }
    }

    public void stopIndexer() {
        indexer.setPower(0);
    }

    // ---------------- STOP ALL ----------------

    public void stopAll() {
        stopShooter();
        stopIntake();
        stopIndexer();
    }

    // ---------------- GETTERS (telemetry) ----------------

    public double getLauncherRPM() {
        return launcherRPM;
    }

    public double getLongRangeRPM() {
        return longRangeRPM;
    }

    public double getShooterVelocity() {
        return launcher.getVelocity();
    }
}
