package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.AutoPathNavigator;

@Autonomous(name = "Red-Close", group = "Autonomous")
public class RedNear extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoPathNavigator.runOpModeRedNear(this);
    }
}
