package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

/**
 * Represents the Teleop OpMode
 */
@TeleOp(name = "Proto", group = "AA_main")
public class Prototyping extends LinearOpMode {
    //private Intake intake;
    //private Indexer indexer;
    //private Actuator actuator;
    private Outtake outtake;
    //private Movement movement;


    @Override
    public void runOpMode() throws InterruptedException {
        // intake = new Intake(hardwareMap);
        // indexer = new Indexer(hardwareMap);
        // actuator = new Actuator(hardwareMap);
        //movement = new Movement(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        GamepadEx gamePadOne = new GamepadEx(gamepad1);
        GamepadEx gamePadTwo = new GamepadEx(gamepad2);


        waitForStart();
        while (opModeIsActive()) {
            gamePadOne.readButtons();
            gamePadTwo.readButtons();
            teleopTick(gamePadOne, gamePadTwo, telemetry);

        } 
    }

    public void teleopTick(GamepadEx padOne, GamepadEx padTwo, Telemetry telemetry) {
        //movement.teleopTick(padOne.getLeftX(),padOne.getLeftY(),padOne.getRightX());//,padOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),telemetry);
        if(padTwo.wasJustPressed(GamepadKeys.Button.A))
        {
            //intake.run(!intake.isRunning());
        }
        if(padTwo.wasJustPressed(GamepadKeys.Button.B))
        {
            //indexer.moveTo(indexer.nextState());
        }
        if(padTwo.wasJustPressed(GamepadKeys.Button.X))
        {

            //actuator.set(!actuator.isActivated());
        }
        if(padTwo.wasJustPressed(GamepadKeys.Button.Y))
        {
            //indexer.quickSpin();
        }
        if(padTwo.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.01){
            outtake.set(5000);
        }
        else {
            outtake.stop();
        }
        if(padTwo.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        {
        }
        else if(padTwo.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
        {
        }

  }
}
