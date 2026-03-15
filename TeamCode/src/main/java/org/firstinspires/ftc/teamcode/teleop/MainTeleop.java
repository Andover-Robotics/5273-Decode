package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

/**
 * MainTeleop is the two-driver teleop OpMode for competition matches.
 *
 * <p>This OpMode wires together the {@link Bot} controller with the FTC
 * runtime in a simple three-phase pattern:
 * <ol>
 *   <li><b>Init</b> – build the Road Runner {@link MecanumDrive}, construct
 *       {@link Bot}, and call {@link Bot#teleopInit()} to reset all subsystems.</li>
 *   <li><b>Start</b> – call {@link Bot#teleopStart()} to position hardware
 *       for the start of the match once the driver presses PLAY.</li>
 *   <li><b>Loop</b> – call {@link Bot#teleopTick()} every iteration until the
 *       match ends or the stop button is pressed.</li>
 * </ol>
 *
 * <p>Telemetry is multiplexed to both the Driver Station and FTC Dashboard via
 * {@link MultipleTelemetry} so live data is visible on both displays during the
 * match.
 *
 * <p>Drive mode is set to <b>robot-centric single-driver</b>
 * ({@code twoMovement = false}): only gamepad 1 controls driving.
 * For a two-movement (operator-also-drives) version see {@link OneDriverTeleop}.
 *
 * <p>Registered as {@code "MainTeleOp"} in the {@code "AA_main"} group so it
 * appears at the top of the OpMode list on the Driver Station.
 */
@TeleOp(name = "MainTeleOp", group = "AA_main")
public class MainTeleop extends LinearOpMode {

    /**
     * Entry point called by the FTC runtime.
     * Initialises hardware, waits for the start signal, then loops.
     *
     * @throws InterruptedException if the OpMode is interrupted while waiting
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Mirror telemetry to FTC Dashboard so tuning values are visible on a laptop/PC.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Construct the Road Runner MecanumDrive at the starting pose (0, 0, 0°).
        // The localizer will be re-zeroed at the human player zone via the X button.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        // Build the Bot controller.  false = gamepad 1 drives (standard two-driver mode).
        Bot bot = new Bot(hardwareMap, telemetry, drive, gamepad1, gamepad2, false);
        bot.teleopInit(); // reset subsystems during init phase

        waitForStart(); // block here until the driver presses the PLAY button

        bot.teleopStart(); // position hardware at match-start positions

        // Main teleop loop: run until the OpMode is stopped.
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick(); // run one complete control loop iteration
        }
    }
}