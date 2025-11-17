package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.AutoPathNavigator;

@Autonomous(name = "Blue-Near", group = "Autonomous")
public class BlueNear extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoPathNavigator.runOpModeBlueNear(this);
    }
}
