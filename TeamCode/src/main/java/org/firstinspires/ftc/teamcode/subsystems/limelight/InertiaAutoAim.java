package org.firstinspires.ftc.teamcode.subsystems.limelight;

public class InertiaAutoAim {
    //Given: Magnitude and direction of Velocity, and range and elevation
    //Get: needed yaw change
    private final static double[] ROBOT_POS = {0,0,0};
    // double[] --> {x,y,z} (y is vertical)

    public InertiaAutoAim() {

    }

    public double getYawDegrees(double[] robotVel, double ballSpeed, double robotYawRad, double goalDistance, double goalElevation) {
        double baseDistance = Math.sqrt(goalDistance * goalDistance - goalElevation * goalElevation);
        double[] goalPos = {baseDistance * Math.sin(robotYawRad),goalElevation,baseDistance * Math.cos(robotYawRad)};
        // goal pos is relative to the robot pos
        double time = (goalDistance / ballSpeed);

        // get the needed yaw change
        double dx = goalPos[0] - (ROBOT_POS[0] + robotVel[0] * time);
        double dz = goalPos[2] - (ROBOT_POS[2] + robotVel[2] * time);
        double yaw = Math.atan2(dz, dx) - robotYawRad;

        return Math.toDegrees(yaw);
    }
}