/*
 * Action methods for use in Auto
 */

package org.firstinspires.ftc.teamcode.auto.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;


public class BotActions {
    private final Intake intake;
    private final Outtake outtake;
    private final Indexer indexer;

    public BotActions(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        intake = new Intake(hardwareMap);
        
        indexer = new Indexer(hardwareMap);

        outtake = new Outtake(hardwareMap);
    }

    public Action actionIntake() {
    // Intakes and rotates indexer
        return new SequentialAction(

        );
    }
    
    public Action actionOuttake() {
    // Rotates Indexer and outtakes all 3
        return new SequentialAction(
        );
    }

    public Action actionPark() {
    // vert slides
        return new ParallelAction(
        );
    }
}
