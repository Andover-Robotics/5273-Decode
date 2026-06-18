package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;


@Config
public class Aimer {
    //bottom left of field is (0,0) (red human player), top right (near red goal) is (144,144)
    //bottom right of field is (144,0) (blue human player), top left (near blue goal) is (0,144)
    //the bot is relocalized at the respective human player zone

    public static double kP = 0.0367;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double filter = 0.867;  // smoothing factor (1 = no filtering, 0 = very heavy smoothing)
    public static double maxIntegral = 1.0;
    public static double deadband = 1;
    private double integral = 0;
    private double lastDerivative = 0.0;
    private double lastError = 0;
    private long lastTimestamp = 0;
    private final MecanumDrive drive;
    //private final InertiaAutoAim inertiaAutoAim;

    //for rpm power
    public static Pose2d targetPoseForRpm = new Pose2d(0, 132, Math.toRadians(90));
    //for bearing
    public static Pose2d targetPoseAim = new Pose2d(0, 132, Math.toRadians(90));
    public static double turretHeight = 11.0; // inches
    public static double targetHeight = 29.5; // inches

    // For Close Shooting
    //how far from the back of the field the aiming point is
    public static double goalBackAimClose = 14; // Only for aiming
    public static double goalBackForRpmClose = 7; // rpm depends on this // normal val: 8
    public static double goalBackAimFar = 14; // Only for aiming
    public static double goalBackForRpmFar = 7; // rpm depends on this // normal val: 8

    // For Far Shooting
    //how far from the side border of the field (where drivers stand) the aiming point is
    public static double goalOutAimClose = 17; // only for aiming
    public static double goalOutForRpmClose = 10; // rpm depends on this // normal val: 15
    public static double goalOutAimFar = 17; // only for aiming
    public static double goalOutForRpmFar = 10; // rpm depends on this // normal val: 15

    public static double centerOfRotationOffsetY = -1.85; // in

    // Relative to center of rotation
    public static double turretOffsetY = -0.41; // in

    private boolean isFarShooting = false;

    public enum Goal {
        RED,
        BLUE
    }

    public static Goal selectedGoal = Goal.RED;

    public Aimer(MecanumDrive mecanumDrive, boolean isFarShooting) {
        this.drive = mecanumDrive;
        //defaults to red goal
        this.isFarShooting = isFarShooting;
        setRedTarget();
        //inertiaAutoAim = new InertiaAutoAim();
    }

    public void setRedTarget(){
        selectedGoal = Goal.RED;

        if (isFarShooting) {
            targetPoseForRpm = new Pose2d(144 - goalOutForRpmFar, 144 - goalBackForRpmFar, Math.toRadians(90));
            targetPoseAim = new Pose2d(144 - goalOutAimFar, 144 - goalBackAimFar, Math.toRadians(90));
        }
        else {
            targetPoseForRpm = new Pose2d(144 - goalOutForRpmClose, 144 - goalBackForRpmClose, Math.toRadians(90));
            targetPoseAim = new Pose2d(144 - goalOutAimClose, 144 - goalBackAimClose, Math.toRadians(90));
        }
    }

    public void setBlueTarget(){
        selectedGoal = Goal.BLUE;

        if (isFarShooting) {
            targetPoseForRpm = new Pose2d(goalOutForRpmFar,144- goalBackForRpmFar, Math.toRadians(90));
            targetPoseAim = new Pose2d(goalOutAimFar,144- goalBackAimFar, Math.toRadians(90));
        }
        else {
            targetPoseForRpm = new Pose2d(goalOutForRpmClose, 144 - goalBackForRpmClose, Math.toRadians(90));
            targetPoseAim = new Pose2d(goalOutAimClose, 144 - goalBackAimClose, Math.toRadians(90));
        }
    }

    public Goal getGoal() {
        return selectedGoal;
    }

    public void relocalize(){
        double botWidth = 14.8;
        double botLength = 17.0;
        // -2.25 because turret not centered
        if(selectedGoal == Goal.RED){
            drive.localizer.setPose(new Pose2d(botWidth/2, botLength/2 + centerOfRotationOffsetY, Math.toRadians(90)));
        } else if (selectedGoal == Goal.BLUE){
            drive.localizer.setPose(new Pose2d(144-botWidth/2, botLength/2 + centerOfRotationOffsetY, Math.toRadians(90)));
        }
    }

    public void localizeForFront(){
        double botWidth = 14.8;
        double botLength = 17.0;
        // -2.25 because turret not centered
        if(selectedGoal == Goal.RED){
            drive.localizer.setPose(new Pose2d(113, 129, Math.toRadians(90)));
        } else if (selectedGoal == Goal.BLUE){
            drive.localizer.setPose(new Pose2d(31, 129, Math.toRadians(90)));
        }
    }

    public double[] calculateLocalizedData() {
        Pose2d robotPose = drive.localizer.getPose();
        double headingRadians = robotPose.heading.toDouble();

        double turretX = robotPose.position.x - (turretOffsetY * Math.sin(headingRadians));
        double turretY = robotPose.position.y + (turretOffsetY * Math.cos(headingRadians));

        double dxForRpm = targetPoseForRpm.position.x - turretX;
        double dyForRpm = targetPoseForRpm.position.y - turretY;

        double dxAim = targetPoseAim.position.x - turretX;
        double dyAim = targetPoseAim.position.y - turretY;

        double horizontalDistanceForRpm = Math.hypot(dxForRpm, dyForRpm);

        double dz = targetHeight - turretHeight;

        double currentHeading = robotPose.heading.toDouble();

        // point-to-point distance, only used for outtake rpm
        double range = Math.hypot(horizontalDistanceForRpm, dz);

        // only used for aim
        double desiredHeading = Math.atan2(dyAim, dxAim);


        double bearing = Math.toDegrees(desiredHeading - currentHeading);
        bearing = angleWrapDegrees(bearing);

        double turnPower = -calculateTurnPowerFromBearing(bearing);
        return new double[]{turnPower, range, bearing, Math.toDegrees(desiredHeading)};
    }

    public double calculateRangeGivenPose(double xPos, double yPos, double headingRadians) {
        Pose2d givenPose = new Pose2d(xPos, yPos, headingRadians);

        double turretX = givenPose.position.x - (turretOffsetY * Math.sin(headingRadians));
        double turretY = givenPose.position.y + (turretOffsetY * Math.cos(headingRadians));

        double dxForRpm = targetPoseForRpm.position.x - turretX;
        double dyForRpm = targetPoseForRpm.position.y - turretY;

        double horizontalDistanceForRpm = Math.hypot(dxForRpm, dyForRpm);

        double dz = targetHeight - turretHeight;

        // point-to-point distance
        double range = Math.hypot(horizontalDistanceForRpm, dz);

        return range;
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

        double error = angleWrapDegrees(bearing);
        if (Math.abs(error) < deadband) {
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

        // output
        double power = kP * error + kI * integral + kD * derivative + feedforward;

        return Math.max(-1, Math.min(1, power));
    }
}