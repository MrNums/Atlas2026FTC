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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * AtlasMecanumDrive
 *
 * Goals:
 * 1) Robot centric and field centric drive
 * 2) Manual rotation only (right stick X). No auto-facing
 * 3) Fix field centric jitter near +/- PI by using UNWRAPPED heading for math
 * 4) Works with both IMU APIs:
 *    - New Control Hub IMU: com.qualcomm.robotcore.hardware.IMU
 *    - Old BNO055 IMU: com.qualcomm.hardware.bosch.BNO055IMU
 * 5) L3 edge press resets heading (re-zero)
 */
public class AtlasMecanumDrive {

    public enum DriveMode { ROBOT_CENTRIC, FIELD_CENTRIC }

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private IMU imuNew = null;
    private BNO055IMU imuOld = null;

    private DriveMode driveMode = DriveMode.FIELD_CENTRIC;

    // Tuning
    public double deadband = 0.05;
    public double turnDeadband = 0.08; // slightly higher to reduce drift-induced spin
    public double maxPower = 0.85;

    // L3 edge detect
    private boolean prevL3 = false;

    // Heading offset (for both IMU types)
    // IMPORTANT: We do NOT wrap heading for the field-centric transform.
    // Wrapping near +/- PI causes sign flips and can jitter when facing "backwards".
    private double headingOffsetRad = 0.0;

    public AtlasMecanumDrive(HardwareMap hw) {
        this(hw, "FL", "FR", "BL", "BR", "imu");
    }

    public AtlasMecanumDrive(
            HardwareMap hw,
            String flName, String frName, String blName, String brName,
            String imuName
    ) {
        frontLeft  = hw.get(DcMotorEx.class, flName);
        frontRight = hw.get(DcMotorEx.class, frName);
        backLeft   = hw.get(DcMotorEx.class, blName);
        backRight  = hw.get(DcMotorEx.class, brName);

        setupMotors();
        initImu(hw, imuName);

        // Set the current robot yaw as "zero"
        resetHeading();
    }

    private void setupMotors() {
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // These directions may need tuning for your wiring/mounting.
        // Forward should drive forward. Strafe should strafe.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initImu(HardwareMap hw, String imuName) {
        // Try new IMU API first
        try {
            imuNew = hw.get(IMU.class, imuName);

            // If your Control Hub is mounted non-standard, you should initialize with hub orientation.
            // Leaving this out assumes standard mounting.
            // imuNew.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(...)));

            // Reset yaw so raw reading starts near 0
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
        driveMode = mode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Call every loop.
     * Left stick: translation
     * Right stick X: rotation (manual only)
     * L3 edge press: reset field-centric heading
     */
    public void drive(Gamepad gp) {
        drive(gp, 1.0, 1.0);
    }

    /**
     * Call every loop with scaling.
     */
    public void drive(Gamepad gp, double driveScale, double turnScale) {
        // L3 edge press -> reset heading
        boolean l3 = gp.left_stick_button;
        if (l3 && !prevL3) resetHeading();
        prevL3 = l3;

        driveScale = Range.clip(driveScale, 0.0, 1.0);
        turnScale  = Range.clip(turnScale,  0.0, 1.0);

        // Translation (left stick)
        double y = -applyDeadband(gp.left_stick_y, deadband) * driveScale; // forward
        double x =  applyDeadband(gp.left_stick_x, deadband) * driveScale; // strafe

        // Rotation (right stick X) - manual only
        double rx = applyDeadband(gp.right_stick_x, turnDeadband) * turnScale;

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            // IMPORTANT: use UNWRAPPED heading for math to avoid +/-PI jitter.
            double heading = getHeadingRadUnwrapped();

            // Rotate joystick vector by -heading
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
     * Mecanum mixing.
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

    /**
     * Re-zero the heading.
     * Works for both IMUs. If new IMU exists, we try resetYaw for cleaner behavior.
     */
    public void resetHeading() {
        double raw = getRawHeadingRad();

        if (imuNew != null) {
            try {
                imuNew.resetYaw();
                raw = 0.0;
            } catch (Throwable ignored) {
                // keep raw, offset logic still works
            }
        }

        headingOffsetRad = raw;
    }
    public void driveRobotCentric(double y, double x, double rx) {
        setMotorPowers(y, x, rx);
    }

    /**
     * Heading used for field-centric math.
     * UNWRAPPED value (no wrapRadians) to avoid discontinuity at +/- PI.
     */
    public double getHeadingRadUnwrapped() {
        return getRawHeadingRad() - headingOffsetRad;
    }

    /**
     * Wrapped heading (nice for telemetry, not recommended for control math).
     */
    public double getHeadingRadWrapped() {
        return wrapRadians(getHeadingRadUnwrapped());
    }

    /**
     * Raw IMU yaw in radians.
     */
    public double getRawHeadingRad() {
        if (imuNew != null) {
            try {
                YawPitchRollAngles ypr = imuNew.getRobotYawPitchRollAngles();
                return ypr.getYaw(AngleUnit.RADIANS);
            } catch (Throwable ignored) { }
        }

        if (imuOld != null) {
            try {
                Orientation o = imuOld.getAngularOrientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
                );
                return o.firstAngle;
            } catch (Throwable ignored) { }
        }

        return 0.0;
    }
    public void driveDirect(double y, double x, double rx) {
        setMotorPowers(y, x, rx);
    }

    public static double applyDeadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }

    private static double wrapRadians(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }
}