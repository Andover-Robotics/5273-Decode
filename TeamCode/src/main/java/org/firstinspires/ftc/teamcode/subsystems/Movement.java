package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

/**
 * Movement controls the four-wheel mecanum drivetrain.
 *
 * <p>A mecanum drivetrain uses wheels with rollers set at 45° to the axle, which
 * allows independent X, Y, and rotational motion.  This class provides both a
 * robot-centric (standard) and a field-centric teleop tick that translates
 * driver stick inputs into per-wheel power values.
 *
 * <p>Motors are wired and configured as follows:
 * <ul>
 *   <li>{@code leftFront} – reversed (motor convention differs from physical direction)</li>
 *   <li>{@code leftBack} – reversed</li>
 *   <li>{@code rightFront} – forward</li>
 *   <li>{@code rightBack} – forward</li>
 * </ul>
 * All four motors use {@link DcMotor.ZeroPowerBehavior#BRAKE} so the robot stops
 * crisply when the driver releases the sticks.
 */
public class Movement {

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** Left-front mecanum wheel motor. */
    private final DcMotor leftFront;

    /** Left-rear mecanum wheel motor. */
    private final DcMotor leftBack;

    /** Right-front mecanum wheel motor. */
    private final DcMotor rightFront;

    /** Right-rear mecanum wheel motor. */
    private final DcMotor rightBack;

    /** Road Runner MecanumDrive reference used for field-centric heading reads
     *  and for updating the pose estimate via the localizer. */
    private final MecanumDrive drive;

    // IMU is currently unused (commented out) because the bot heading is
    // obtained from the Road Runner localizer instead.
    // private final IMU imu;

    // -----------------------------------------------------------------------
    // Scaling constants
    // -----------------------------------------------------------------------

    /** Multiplier applied to both the forward/axial and strafe/lateral
     *  components.  A value of 1.0 means full stick deflection produces
     *  full motor power in those axes. */
    private final double STRAFE_MULTIPLIER = 1.0;

    /** Multiplier applied to the rotational/yaw component.  Reduced to 0.8 to
     *  make turning slightly less sensitive than strafing/driving. */
    private final double ROTATION_MULTIPLIER = 0.8;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Initialises the Movement subsystem by fetching all four drive motors from
     * the hardware map, configuring their directions, and setting their zero-
     * power behavior to BRAKE.
     *
     * @param map   the robot's hardware map, non-null
     * @param drive the Road Runner MecanumDrive used to read heading for
     *              field-centric mode and to keep the pose estimate updated
     */
    public Movement(@NonNull HardwareMap map, MecanumDrive drive){
        // Retrieve each motor by the name registered in the hardware configuration.
        leftFront  = map.get(DcMotor.class, "leftFront");
        leftBack   = map.get(DcMotor.class, "leftBack");
        rightFront = map.get(DcMotor.class, "rightFront");
        rightBack  = map.get(DcMotor.class, "rightBack");

        this.drive = drive; // store Road Runner drive for localizer access
        // this.imu = drive.lazyImu.get(); // IMU path: currently disabled

        // Reverse left-side motors so that all motors spin "forward" with a
        // positive power value, given the physical wiring of this robot.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Enable BRAKE mode on all motors so the robot holds its position when
        // power is cut, instead of coasting unpredictably.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------------------------------------------------
    // Teleop drive methods
    // -----------------------------------------------------------------------

    /**
     * Robot-centric mecanum drive tick called every loop iteration during teleop.
     *
     * <p>Translates three driver inputs (forward/axial, strafe/lateral, rotate/yaw)
     * into individual wheel powers using the standard mecanum kinematics matrix.
     * A "denominator" normalisation ensures at least one wheel always reaches
     * full power when the driver pushes to the stick limit.
     *
     * <p>An additional {@code turnCorrection} term from the AprilTag aimer is
     * blended into the yaw component to keep the robot pointed at the goal
     * while the driver manoeuvres.
     *
     * @param leftStickX     horizontal axis of the left stick (-1 = strafe left,  +1 = strafe right)
     * @param leftStickY     vertical axis of the left stick   (-1 = drive back,   +1 = drive forward)
     * @param rightStickX    horizontal axis of the right stick(-1 = turn left,    +1 = turn right)
     * @param turnCorrection additional rotation correction from vision aiming (can be 0 to disable)
     */
    public void teleopTick(double leftStickX, double leftStickY, double rightStickX, double turnCorrection){
        // Scale the axial (forward) and lateral (strafe) components by the multiplier.
        double axial   = leftStickY  * STRAFE_MULTIPLIER;   // forward/backward
        double lateral = leftStickX  * STRAFE_MULTIPLIER;   // left/right strafe
        // Blend the driver's rotation request with the auto-aim correction.
        double yaw     = rightStickX * ROTATION_MULTIPLIER + turnCorrection;

        // Mecanum kinematics: each wheel's power = sum/difference of axial, lateral, yaw.
        double leftFrontPower  = axial + lateral + yaw;   // front-left: all three add
        double rightFrontPower = axial - lateral - yaw;   // front-right: lateral & yaw subtract
        double leftBackPower   = axial - lateral + yaw;   // back-left: lateral subtracts, yaw adds
        double rightBackPower  = axial + lateral - yaw;   // back-right: yaw subtracts

        // Normalise: find the largest absolute wheel power.  If it exceeds 1.0
        // we divide all powers by it to keep them in [-1, 1] while preserving
        // their ratios (smooth joystick "feel").
        double denominator = Math.max(1.0, Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw));

        leftFrontPower  /= denominator;
        rightFrontPower /= denominator;
        leftBackPower   /= denominator;
        rightBackPower  /= denominator;

        // Write the computed powers to the physical motors.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Field-centric mecanum drive tick called every loop iteration during teleop.
     *
     * <p><b>NOTE: Field-centric mode is currently not fully functional with the
     * AprilTag aiming system.</b>  The heading source (Road Runner localizer)
     * must have been initialised/re-zeroed before calling this method.
     *
     * <p>In field-centric mode the driver's forward direction always corresponds
     * to the physical field "north" regardless of the robot's heading.  This is
     * achieved by rotating the stick input vector by the negative of the robot's
     * current heading angle.
     *
     * @param leftStickX     horizontal axis of the left stick  (field-relative strafe)
     * @param leftStickY     vertical axis of the left stick    (field-relative forward)
     * @param rightStickX    horizontal axis of the right stick (rotation)
     * @param turnCorrection additional rotation correction from vision aiming
     * @param start          when {@code true} resets the localizer pose to (0, 0, 0);
     *                       typically mapped to the "Start" button on an Xbox-style
     *                       controller to re-zero field orientation on demand
     */
    // NOTE DOESN'T WORK WITH APRILTAG RN
    public void teleopTickFieldCentric(double leftStickX, double leftStickY, double rightStickX, double turnCorrection, boolean start){
        double axial   = leftStickY  * STRAFE_MULTIPLIER;   // field-relative forward
        double lateral = leftStickX  * STRAFE_MULTIPLIER;   // field-relative strafe
        double yaw     = rightStickX * ROTATION_MULTIPLIER + turnCorrection;

        // If the driver requests a heading reset, zero the localizer pose so that
        // the current robot orientation becomes the new field "forward".
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (start) {
            drive.localizer.setPose(new Pose2d(0, 0, 0)); // re-zero field origin at current robot pose
        }

        // Read the robot's heading angle (in radians) from the Road Runner localizer.
        double botHeading = drive.localizer.getPose().heading.log(); // imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the driver's input vector counter to the robot's heading so that
        // "forward" on the stick always means "field north" regardless of robot orientation.
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        // Compensate for imperfect strafing (mecanum wheels tend to slip laterally;
        // scaling by 1.1 empirically corrects for this offset on this robot).
        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Normalise: largest absolute component (or 1 if all are ≤ 1) to keep
        // powers within [-1, 1] while maintaining their ratio.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
        double frontLeftPower  = (rotY + rotX + yaw) / denominator;
        double backLeftPower   = (rotY - rotX + yaw) / denominator;
        double frontRightPower = (rotY - rotX - yaw) / denominator;
        double backRightPower  = (rotY + rotX - yaw) / denominator;

        // Write field-centric powers to each wheel.
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    // -----------------------------------------------------------------------
    // Motor accessors (used by autonomous and diagnostics)
    // -----------------------------------------------------------------------

    /** @return the left-front drive motor hardware object */
    public DcMotor getLeftFront() {
        return leftFront;
    }

    /** @return the left-rear drive motor hardware object */
    public DcMotor getLeftBack() {
        return leftBack;
    }

    /** @return the right-front drive motor hardware object */
    public DcMotor getRightFront() {
        return rightFront;
    }

    /** @return the right-rear drive motor hardware object */
    public DcMotor getRightBack() {
        return rightBack;
    }

    // -----------------------------------------------------------------------
    // Pose helper
    // -----------------------------------------------------------------------

    /**
     * Updates the Road Runner localizer's stored pose estimate.
     * Useful after a known field event (e.g. the robot presses against a wall)
     * that allows a hard reset of the position estimate without performing a
     * full relocalization scan.
     *
     * @param newPose the new absolute pose to assign to the localizer
     */
    public void setPose(Pose2d newPose) {
        drive.localizer.setPose(newPose); // propagate the new pose into the Road Runner localizer
    }
}


