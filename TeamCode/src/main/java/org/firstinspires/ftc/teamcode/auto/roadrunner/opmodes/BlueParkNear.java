package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.AutoPathNavigator;

@Autonomous(name = "BluePark-Near", group = "Autonomous")
public class BlueParkNear extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoPathNavigator.runOpModeBlueParkNear(this);
    }
}
