package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.ConcreteLazyImu;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Movement;

import org.openftc.easyopencv.*;

@TeleOp(name = "AprilTagTester", group = "testing")
public class AprilTagTester extends LinearOpMode {
    OpenCvCamera camera;
    private long lastAimUpdateTime = 0;
    private double lastTurnCorrection = 0;
    private boolean fieldCentric = false;
    private static final long AIM_UPDATE_INTERVAL_MS = 50;  // update every 50 ms (~20 Hz)
    private static int goalTagID;

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTag aprilTag = new AprilTag(hardwareMap,telemetry);
        ConcreteLazyImu concreteImu = new ConcreteLazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        Movement movement = new Movement(hardwareMap, concreteImu);
        AprilTagAimer aprilAimer = new AprilTagAimer(hardwareMap, movement.getImu(), movement.getTwoDeadWheelLocalizer());
        GamepadEx gamePadOne = new GamepadEx(gamepad1);
        GamepadEx gamePadTwo = new GamepadEx(gamepad2);
        boolean continuousAprilTagLock = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            double turnCorrection = 0;
            if (continuousAprilTagLock) {
                long currentTime = System.currentTimeMillis();

                // Run scan + PID only every AIM_UPDATE_INTERVAL_MS
                if (currentTime - lastAimUpdateTime >= AIM_UPDATE_INTERVAL_MS) {
                    lastAimUpdateTime = currentTime;

                    aprilTag.scanGoalTag();
                    double bearing = aprilTag.getBearing();

                    if (!Double.isNaN(bearing)) {
                        lastTurnCorrection = aprilAimer.calculateTurnPowerFromBearing(bearing);
                    } else {
                        lastTurnCorrection = aprilAimer.calculateLocalizedTurnPower(goalTagID)[0];
                    }
                }

                // turnCorrection = 0.9 * lastTurnCorrection; - don't want this
            } else {
                turnCorrection = 0;
            }

            if (fieldCentric) {
                movement.teleopTickFieldCentric(gamePadOne.getLeftX(), gamePadOne.getLeftY(), gamePadOne.getRightX(), turnCorrection, true);
            }
            else {
                movement.teleopTick(gamePadOne.getLeftX(), gamePadOne.getLeftY(), gamePadOne.getRightX(), turnCorrection);
            }

            if (gamePadOne.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                fieldCentric = true;
            }
            if (gamePadOne.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                fieldCentric = false;
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.Y)) {
                aprilTag.scanObeliskTag();
                telemetry.addData("This is probably only for auto,", "as we can just memorize the 3 possible patterns for teleop");
                telemetry.addData("Obelisk apriltag ID: ", aprilTag.getObeliskId());
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.A)) {
                continuousAprilTagLock = true;
                aprilTag.setCurrentCameraScannedId(0);
            }

            if (continuousAprilTagLock) {
                telemetry.addData("Button A to update", "telemetry");
                telemetry.addData("Continuously locked in on", "apriltag");
                telemetry.addData("Last detected tag ID", aprilTag.getCurrentId());
                telemetry.addData("Goal tag bearing", aprilTag.getBearing());
                telemetry.addData("Goal tag elevation", aprilTag.getElevation());
                telemetry.addData("Goal tag range", aprilTag.getRange());
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.B)) {
                continuousAprilTagLock = false;

                telemetry.addData("Stopped continuous lock in on", "apriltag");
            }

            if(gamePadTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                aprilTag.setPipeline(0);
                telemetry.addData("Set to", "Blue Alliance") ;
            }
            if(gamePadTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                aprilTag.setPipeline(1);
                telemetry.addData("Set to", "Red Alliance");
            }

            telemetry.update();
        }
    }
}