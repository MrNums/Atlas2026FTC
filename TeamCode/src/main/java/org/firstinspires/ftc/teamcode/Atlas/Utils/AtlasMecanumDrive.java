package org.firstinspires.ftc.teamcode.Atlas.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
    AtlasMecanumDrive
    Supports Robot Centric and Field Centric drive.
    Works with both IMU APIs:
    1) New Control Hub IMU: com.qualcomm.robotcore.hardware.IMU
    2) Older BNO055 IMU: com.qualcomm.hardware.bosch.BNO055IMU

    Features
    - Field centric with IMU heading
    - Robot centric
    - Reset orientation with L3 (left stick button) edge press
 */
public class AtlasMecanumDrive {

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private IMU imuNew = null;
    private BNO055IMU imuOld = null;

    private double headingOffsetRad = 0.0;
    private boolean prevL3 = false;

    private DriveMode driveMode = DriveMode.FIELD_CENTRIC;

    public double deadband = 0.05;
    public double turnDeadband = 0.05;
    public double maxPower = 0.85;
    public AtlasMecanumDrive(HardwareMap hw) {
        this(hw, "frontLeft", "frontRight", "backLeft", "backRight", "imu");
    }
    public AtlasMecanumDrive(
            HardwareMap hw,
            String flName, String frName, String blName, String brName,
            String imuName
    ) {
        frontLeft = hw.get(DcMotorEx.class, flName);
        frontRight = hw.get(DcMotorEx.class, frName);
        backLeft = hw.get(DcMotorEx.class, blName);
        backRight = hw.get(DcMotorEx.class, brName);

        setupMotors();
        initImu(hw, imuName);
        resetHeading();
    }

    private void setupMotors() {
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        // If your robot drives backwards or strafes wrong, fix direction here first.
    }

    private void initImu(HardwareMap hw, String imuName) {
        // Try new IMU first (newer SDK, newer Control Hub)
        try {
            imuNew = hw.get(IMU.class, imuName);

            // Optional, but recommended: set the hub orientation if your Control Hub is mounted differently.
            // If you do not set it, heading might be rotated by 90 or 180 degrees.
            // Example (only if you know your mounting):
            // IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
            //         RevHubOrientationOnRobot.LogoFacingDirection.UP,
            //         RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            // ));
            // imuNew.initialize(params);

            // If not initialized elsewhere, this is still safe to call.
            imuNew.resetYaw();
            return;
        } catch (Throwable ignored) {
            imuNew = null;
        }

        // Fallback to old BNO055 IMU
        try {
            imuOld = hw.get(BNO055IMU.class, imuName);
            BNO055IMU.Parameters p = new BNO055IMU.Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imuOld.initialize(p);
        } catch (Throwable ignored) {
            imuOld = null;
        }
    }

    public void setDriveMode(DriveMode mode) {
        this.driveMode = mode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Call every loop in TeleOp.
     * Uses:
     *  left stick: translation
     *  right stick x: rotation
     *  L3 edge press: reset heading
     */
    public void drive(Gamepad gp) {
        // L3 to reset heading
        boolean l3 = gp.left_stick_button;
        if (l3 && !prevL3) {
            resetHeading();
        }
        prevL3 = l3;

        double y = -applyDeadband(gp.left_stick_y, deadband); // forward
        double x = applyDeadband(gp.left_stick_x, deadband);  // strafe
        double rx = applyDeadband(gp.right_stick_x, turnDeadband); // turn

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double heading = getHeadingRad();

            // Rotate the joystick vector by negative heading
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);

            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;

            setMotorPowers(rotY, rotX, rx);
        } else {
            setMotorPowers(y, x, rx);
        }
    }

    /**
     * Robot centric helper.
     */
    private void setMotorPowers(double y, double x, double rx) {
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));

        fl = (fl / max) * maxPower;
        fr = (fr / max) * maxPower;
        bl = (bl / max) * maxPower;
        br = (br / max) * maxPower;

        frontLeft.setPower(Range.clip(fl, -1.0, 1.0));
        frontRight.setPower(Range.clip(fr, -1.0, 1.0));
        backLeft.setPower(Range.clip(bl, -1.0, 1.0));
        backRight.setPower(Range.clip(br, -1.0, 1.0));
    }

    public void resetHeading() {
        double raw = getRawHeadingRad();

        // If new IMU is present, you can also call resetYaw.
        // We still keep offset logic so it works consistently.
        if (imuNew != null) {
            try {
                imuNew.resetYaw();
                raw = 0.0;
            } catch (Throwable ignored) {
                // offset will handle it
            }
        }

        headingOffsetRad = raw;
    }

    public double getHeadingRad() {
        double raw = getRawHeadingRad();
        return wrapRadians(raw - headingOffsetRad);
    }

    public double getRawHeadingRad() {
        // New IMU API
        if (imuNew != null) {
            try {
                YawPitchRollAngles ypr = imuNew.getRobotYawPitchRollAngles();
                return ypr.getYaw(AngleUnit.RADIANS);
            } catch (Throwable ignored) { }
        }

        // Old BNO055 API
        if (imuOld != null) {
            try {
                Orientation o = imuOld.getAngularOrientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
                );
                return o.firstAngle; // yaw
            } catch (Throwable ignored) { }
        }

        return 0.0;
    }

    private static double applyDeadband(double v, double db) {
        return Math.abs(v) < db ? 0.0 : v;
    }

    private static double wrapRadians(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }
}