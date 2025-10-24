package org.firstinspires.ftc.teamcode.subsystems;

public class InertiaAutoAim {
    //Given: Magnitude and direction of Velocity, and range and elevation
    //Get: needed yaw change
    private final static double[] robotPos = {0,0,0};
    // double[] --> {x,y,z} (y is vertical)
    public double getYawDegrees(double[] robotVel, double ballSpeed, double robotYawRad, double goalDistance, double goalElevation) {
        double baseDistance = Math.sqrt(goalDistance * goalDistance - goalElevation * goalElevation);
        double[] goalPos = {baseDistance * Math.sin(robotYawRad),goalElevation,baseDistance * Math.cos(robotYawRad)};
        // goal pos is relative to the robot pos
        double time = (goalDistance / ballSpeed);

        // get the needed yaw change
        double dx = goalPos[0] - (robotPos[0] + robotVel[0] * time);
        double dz = goalPos[2] - (robotPos[2] + robotVel[2] * time);
        double yaw = Math.atan2(dz, dx) - robotYawRad;

        return Math.toDegrees(yaw);
    }
}
