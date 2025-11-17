package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.AutoPathNavigator;

@Autonomous(name = "Red-Far", group = "Autonomous")
public class RedFar extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoPathNavigator.runOpModeRedFar(this);
    }
}
