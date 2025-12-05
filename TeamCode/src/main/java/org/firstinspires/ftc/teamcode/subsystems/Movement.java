package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Represents the drivetrain.
 */
public class Movement {
    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    private final IMU imu;
    private final double STRAFE_MULTIPLIER = 1.0, ROTATION_MULTIPLIER = 0.8;

    /**
     * Initializes a Movement instance.
     * @param map {@link com.qualcomm.robotcore.hardware.HardwareMap}
     */
    public Movement(@NonNull HardwareMap map){
        leftFront = map.get(DcMotor.class, "leftFront");
        leftBack = map.get(DcMotor.class, "leftBack");
        rightFront = map.get(DcMotor.class, "rightFront");
        rightBack = map.get(DcMotor.class, "rightBack");

        imu = map.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // tick for teleop

    public void teleopTick(double leftStickX, double leftStickY, double rightStickX, double turnCorrection){
        double axial = -leftStickY * STRAFE_MULTIPLIER;
        double lateral = -leftStickX * STRAFE_MULTIPLIER;
        double yaw = -(rightStickX * ROTATION_MULTIPLIER + turnCorrection);

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // For smoother joystick movement
        double denominator = Math.max(1.0, Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw));

        leftFrontPower  /= denominator;
        rightFrontPower /= denominator;
        leftBackPower   /= denominator;
        rightBackPower  /= denominator;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    // NOTE DOESN'T WORK WITH APRILTAG RN
    public void teleopTickFieldCentric(double leftStickX, double leftStickY, double rightStickX, double turnCorrection, boolean start){
        double axial = -leftStickY * STRAFE_MULTIPLIER;
        double lateral = -leftStickX * STRAFE_MULTIPLIER;
        double yaw = -(rightStickX * ROTATION_MULTIPLIER + turnCorrection);

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (start) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
        double frontLeftPower = (rotY + rotX + yaw) / denominator;
        double backLeftPower = (rotY - rotX + yaw) / denominator;
        double frontRightPower = (rotY - rotX - yaw) / denominator;
        double backRightPower = (rotY + rotX - yaw) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public DcMotor getLeftFront() {
        return leftFront;
    }

    public DcMotor getLeftBack() {
        return leftBack;
    }

    public DcMotor getRightFront() {
        return rightFront;
    }

    public DcMotor getRightBack() {
        return rightBack;
    }

    public IMU getImu() {
        return imu;
    }
}

