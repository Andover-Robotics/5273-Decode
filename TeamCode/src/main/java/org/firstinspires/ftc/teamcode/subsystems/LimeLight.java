package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


// to configure pipline for apriltags you have to configure the physicial limelight via the limelight web interface

public class LimeLight {
    private Limelight3A limelight;
    private double pitch,yaw,roll;
    LimeLight(HardwareMap hardwareMap)
    {
        init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Select pipeline 0, adjust if needed (configure in interface)
    }
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
    public double getPitch()
    {
        return pitch;
    }
    public double getYaw()
    {
        return yaw;
    }
    public double getRoll()
    {
        return roll;
    }
    public void start()
    {
        limelight.start();            // Start the vision processing loop
        LLResult result = limelight.getLatestResult();
        Pose3D botPose = result.getBotpose();
        pitch = botPose.getOrientation().getPitch();
        yaw = botPose.getOrientation().getYaw();
        roll = botPose.getOrientation().getRoll();
    }














}
