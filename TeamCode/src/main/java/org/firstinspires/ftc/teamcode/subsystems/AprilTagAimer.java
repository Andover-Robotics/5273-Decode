package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Config
public class AprilTagAimer {
    public static double kP = 0.025;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0067;
    public static double filter = 0.867;  // smoothing factor (1 = no filtering, 0 = very heavy smoothing)
    public static double maxIntegral = 1.0;
    private double integral = 0;
    private double lastDerivative = 0.0;
    private double lastError = 0;
    private long lastTimestamp = 0;
    private final IMU imu;

    /* When and why to tune these
    P (Proportional) Changes core power of turns, its proportional
    I (Integral) Maybe rarely if robot consistently falls short of the target (steady-state error).
    D (Derivative) Increase to dampen motion and reduce overshoot. Good for smoothing quick heading corrections.
    F (Feedforward)	Maybe, its a constant, increase to help overcome drivetrain static friction and give better response when error is small.
    */
    public AprilTagAimer(HardwareMap hardwareMap) {
        // Initialize IMU directly
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    public double calculateIMUTurnPower(int tagID) {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        if (tagID == 20) {
            return calculateTurnPowerFromBearing(135 - Yaw);
        }
        else if (tagID == 24) {
            return calculateTurnPowerFromBearing(45 - Yaw);
        }

        return 0;
    }

    private double angleWrapDegrees(double angle) {
        return (angle + 180) % 360 - 180;
    }

    public double calculateTurnPowerFromBearing(double bearing) {
        // If apriltag lost - reset PID state and return no correction
        if (Double.isNaN(bearing)) {
            integral = 0;
            lastError = 0;
            lastDerivative = 0;
            lastTimestamp = 0;
            return 0.0;
        }

        double error = -angleWrapDegrees(bearing);
        if (Math.abs(error) < 1.0) {
            integral = 0;
            lastError = error;
            lastDerivative = 0;
            return 0;
        }

        // Time diff in seconds
        long currentTime = System.currentTimeMillis();
        double deltaTime;

        if (lastTimestamp == 0) {
            deltaTime = 0.01;  // assume 10 ms on first loop
        } else {
            deltaTime = (currentTime - lastTimestamp) / 1000.0;
        }
        lastTimestamp = currentTime;

        // Don't consider > 100 ms or < 5ms
        deltaTime = Math.max(Math.min(deltaTime, 0.1), .005);

        // Integral accumulation
        integral += error * deltaTime;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        // Derivative with smoothing
        double rawDerivative = (error - lastError) / deltaTime;
        double derivative = (1 - filter) * lastDerivative + filter * rawDerivative;
        lastDerivative = derivative;
        lastError = error;

        // Gets proper direction
        double feedforward = Math.signum(error) * kF;

        // PIDF output
        double power = kP * error + kI * integral + kD * derivative + feedforward;

        return Math.max(-1, Math.min(1, power));
    }
}