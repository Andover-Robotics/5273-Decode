package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;


// to configure pipline for apriltags you have to configure the physicial limelight via the limelight web interface

public class LimeLight {
    private Limelight3A limelight;
    private double xOffset, yOffset, size;
    private boolean hasPose;
    public LimeLight(HardwareMap hardwareMap)
    {
        init(hardwareMap);
        limelight.pipelineSwitch(0); // adjust in the limelight app/interface thingy 

    }
    public void start()
    {
        limelight.start();
    }
    private void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }
    public void update() {
        LLResult result = limelight.getLatestResult();

        hasPose = false;
        if (result != null && result.isValid()) {
            xOffset = result.getTx();
            yOffset = result.getTy();
            size = result.getTa();
            hasPose = true;
        }
    }
    public boolean detected() { return hasPose; }
    public double getXOffset()
    {
        return xOffset;
    }
    public double getYOffset()
    {
        return yOffset;
    }
    public double getSize()
    {
        return size;
    }















}
