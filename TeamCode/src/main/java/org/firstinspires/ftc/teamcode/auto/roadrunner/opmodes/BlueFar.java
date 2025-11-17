package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.AutoPathNavigator;

@Autonomous(name = "Blue-Far", group = "Autonomous")
public class BlueFar extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoPathNavigator.runOpModeBlueFar(this);
    }
}
