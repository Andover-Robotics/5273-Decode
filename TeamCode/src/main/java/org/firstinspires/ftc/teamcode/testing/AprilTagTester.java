package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Movement;

@TeleOp(name = "AprilTagTester", group = "AA_main")
public class AprilTagTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTag aprilTag = new AprilTag(hardwareMap);
        Movement movement = new Movement(hardwareMap);
        // Outtake outtake = new Outtake(hardwareMap);
        AprilTagAimer aprilAimer = new AprilTagAimer(hardwareMap, movement); // Pass in outtake when possible
        GamepadEx gamePadOne = new GamepadEx(gamepad1);
        GamepadEx gamePadTwo = new GamepadEx(gamepad2);

        waitForStart();
        while (opModeIsActive()) {
            gamePadOne.readButtons();
            gamePadTwo.readButtons();

            movement.teleopTick(gamePadOne.getLeftX(),gamePadOne.getLeftY(),gamePadOne.getRightX());

            telemetry.addData("X:", "Scan obelisk apriltag");
            telemetry.addData("A", "Test april tag aimer with blue alliance apriltag");
            telemetry.addData("B", "Test april tag aimer with red alliance apriltag");
            telemetry.update();

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.A)) {
                aprilTag.setGoalTagID(20);
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                aprilAimer.startTurnToAprilTag(bearing);

                while (aprilAimer.updateTurn()) {
                    telemetry.addData("Turning towards angle", bearing);
                    telemetry.update();
                    sleep(10);
                }

                telemetry.addData("Finished", "locking on to apriltag");
                telemetry.update();
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.B)) {
                aprilTag.setGoalTagID(24);
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                aprilAimer.startTurnToAprilTag(bearing);

                while (aprilAimer.updateTurn()) {
                    telemetry.addData("Turning towards angle", bearing);
                    telemetry.update();
                    sleep(10);
                }

                telemetry.addData("Finished", "locking on to apriltag");
                telemetry.update();
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.X)) {
                aprilTag.scanObeliskTag();
                telemetry.addData("This is probably only for auto,", "as we can just memorize the 3 possible patterns for teleop");
                telemetry.addData("Obelisk apriltag ID: ", aprilTag.getObeliskId());
                telemetry.update();
            }
        }
    }
}