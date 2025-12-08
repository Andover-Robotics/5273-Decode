package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


// to configure pipline for apriltags you have to configure the physicial limelight via the limelight web interface

public class LimeLight {
    private Limelight3A limelight;
    private double pitch,yaw,roll;
    LLResult result;
    private boolean hasPose;
    LimeLight(HardwareMap hardwareMap)
    {
        init(hardwareMap);
        limelight.pipelineSwitch(0); // adjust in the limelight app/interface thingy 

    }
    public void start()
    {
        limelight.start();
    }
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }
    public void update() {
        LLResult result = limelight.getLatestResult();

        hasPose = false;

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose(); // or getBotpose_MT2()
            if (botPose != null) {
                pitch = botPose.getOrientation().getPitch();
                yaw   = botPose.getOrientation().getYaw();
                roll  = botPose.getOrientation().getRoll();
                hasPose = true;
            }
        }
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















}
