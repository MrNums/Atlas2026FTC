package org.firstinspires.ftc.teamcode.Atlas.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AtlasMecanumDrive {

    public enum DriveMode { ROBOT_CENTRIC, FIELD_CENTRIC }

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private IMU imuNew;
    private BNO055IMU imuOld;

    private DriveMode driveMode = DriveMode.FIELD_CENTRIC;
    private double headingOffsetRad;

    private boolean lastLeftStickButton;

    public double deadband = 0.05;
    public double turnDeadband = 0.08;
    public double maxPower = 0.85;

    public AtlasMecanumDrive(HardwareMap hw) {
        this(hw, "FL", "FR", "BL", "BR", "imu");
    }

    public AtlasMecanumDrive(HardwareMap hw, String flName, String frName, String blName, String brName, String imuName) {
        frontLeft = hw.get(DcMotorEx.class, flName);
        frontRight = hw.get(DcMotorEx.class, frName);
        backLeft = hw.get(DcMotorEx.class, blName);
        backRight = hw.get(DcMotorEx.class, brName);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        try {
            imuNew = hw.get(IMU.class, imuName);
            imuNew.resetYaw();
        } catch (Throwable ignored) {
            imuNew = null;
        }

        if (imuNew == null) {
            try {
                imuOld = hw.get(BNO055IMU.class, imuName);
                BNO055IMU.Parameters params = new BNO055IMU.Parameters();
                params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imuOld.initialize(params);
            } catch (Throwable ignored) {
                imuOld = null;
            }
        }

        resetHeading();
    }

    public void drive(Gamepad gp) {
        drive(gp, 1.0, 1.0);
    }

    public void drive(Gamepad gp, double driveScale, double turnScale) {
        boolean leftStickPressed = gp.left_stick_button;
        if (leftStickPressed && !lastLeftStickButton) resetHeading();
        lastLeftStickButton = leftStickPressed;

        double y = -applyDeadband(gp.left_stick_y, deadband) * Range.clip(driveScale, 0, 1);
        double x = applyDeadband(gp.left_stick_x, deadband) * Range.clip(driveScale, 0, 1);
        double turn = applyDeadband(gp.right_stick_x, turnDeadband) * Range.clip(turnScale, 0, 1);

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double heading = getHeadingRadUnwrapped();
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);

            double fieldX = x * cos - y * sin;
            double fieldY = x * sin + y * cos;
            setMotorPowers(fieldY, fieldX, turn);
        } else {
            setMotorPowers(y, x, turn);
        }
    }

    public void driveDirect(double y, double x, double turn) {
        setMotorPowers(y, x, turn);
    }

    public void driveRobotCentric(double y, double x, double turn) {
        setMotorPowers(y, x, turn);
    }

    public void resetHeading() {
        double rawHeading = getRawHeadingRad();
        if (imuNew != null) {
            try {
                imuNew.resetYaw();
                rawHeading = 0;
            } catch (Throwable ignored) {
                // use offset method below if reset fails
            }
        }
        headingOffsetRad = rawHeading;
    }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public double getHeadingRadUnwrapped() {
        return getRawHeadingRad() - headingOffsetRad;
    }

    public double getHeadingRadWrapped() {
        return wrapRadians(getHeadingRadUnwrapped());
    }

    public double getRawHeadingRad() {
        if (imuNew != null) {
            try {
                return imuNew.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            } catch (Throwable ignored) {
                // fallback below
            }
        }

        if (imuOld != null) {
            try {
                Orientation orientation = imuOld.getAngularOrientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
                );
                return orientation.firstAngle;
            } catch (Throwable ignored) {
                // return 0 below
            }
        }

        return 0;
    }

    private void setMotorPowers(double y, double x, double turn) {
        double fl = y + x + turn;
        double bl = y - x + turn;
        double fr = y - x - turn;
        double br = y + x - turn;

        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));

        frontLeft.setPower(Range.clip((fl / scale) * maxPower, -1, 1));
        backLeft.setPower(Range.clip((bl / scale) * maxPower, -1, 1));
        frontRight.setPower(Range.clip((fr / scale) * maxPower, -1, 1));
        backRight.setPower(Range.clip((br / scale) * maxPower, -1, 1));
    }

    public static double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }

    private static double wrapRadians(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
