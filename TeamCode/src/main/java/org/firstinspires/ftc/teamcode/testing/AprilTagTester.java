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
        AprilTagAimer aprilAimer = new AprilTagAimer(hardwareMap);
        Movement movement = new Movement(hardwareMap);
        GamepadEx gamePadOne = new GamepadEx(gamepad1);
        GamepadEx gamePadTwo = new GamepadEx(gamepad2);
        boolean continuousAprilTagLock = false;

        telemetry.addData("Gamepad 2 Y:", "Scan obelisk apriltag");
        telemetry.addData("Gamepad 2 A:", "Continuously lock into apriltag");
        telemetry.addData("Gamepad 2 B:", "Stop continuously locking into apriltag");
        telemetry.addData("Gamepad 2 Left Bumper", "Set to blue alliance apriltag");
        telemetry.addData("Gamepad 2 Right Bumper", "Set to red alliance apriltag");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            gamePadOne.readButtons();
            gamePadTwo.readButtons();

            double turnCorrection;
            if (continuousAprilTagLock) {
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                if (Double.isNaN(bearing)) {
                    turnCorrection = 0;
                } else {
                    turnCorrection = aprilAimer.calculateTurnPowerToBearing(bearing);
                }
            }
            else {
                turnCorrection = 0;
            }

            movement.teleopTick(gamePadOne.getLeftX(), gamePadOne.getLeftY(), gamePadOne.getRightX(), turnCorrection);

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.Y)) {
                aprilTag.scanObeliskTag();
                telemetry.addData("This is probably only for auto,", "as we can just memorize the 3 possible patterns for teleop");
                telemetry.addData("Obelisk apriltag ID: ", aprilTag.getObeliskId());
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.A)) {
                continuousAprilTagLock = true;

                telemetry.addData("Continuously locked in on", "apriltag");
                telemetry.addData("Goal tag bearing", aprilTag.getBearing());
                telemetry.addData("Goal tag elevation", aprilTag.getElevation());
                telemetry.addData("Goal tag range", aprilTag.getRange());
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.B)) {
                continuousAprilTagLock = false;

                telemetry.addData("Stopped continuous lock in on", "apriltag");
            }

            if(gamePadTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                aprilTag.setGoalTagID(20);
                telemetry.addData("Set to", "Blue Alliance") ;
            }
            if(gamePadTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                aprilTag.setGoalTagID(24);
                telemetry.addData("Set to", "Red Alliance");
            }

            telemetry.update();
        }
    }
}
