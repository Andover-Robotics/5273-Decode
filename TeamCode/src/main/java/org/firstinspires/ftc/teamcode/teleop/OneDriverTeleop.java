package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

/**
 * OneDriverTeleop is an alternative teleop OpMode intended for single-driver
 * practice sessions or situations where only one gamepad is available.
 *
 * <p>The structure mirrors {@link MainTeleop} exactly; the only difference is
 * the OpMode name registered with the FTC runtime ({@code "OneDriver"} instead
 * of {@code "MainTeleOp"}).  Both OpModes pass {@code twoMovement = false} to
 * {@link Bot}, meaning only gamepad 1 controls driving.
 *
 * <p>This OpMode also appears in the {@code "AA_main"} group on the Driver
 * Station for quick access.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li><b>Init</b> – construct {@link MecanumDrive} and {@link Bot};
 *       call {@link Bot#teleopInit()}.</li>
 *   <li><b>Start</b> – call {@link Bot#teleopStart()} once the match begins.</li>
 *   <li><b>Loop</b> – call {@link Bot#teleopTick()} each iteration.</li>
 * </ol>
 */
@TeleOp(name = "OneDriver", group = "AA_main")
public class OneDriverTeleop extends LinearOpMode {

    /**
     * Entry point called by the FTC runtime.
     * Initialises hardware, waits for the start signal, then loops.
     *
     * @throws InterruptedException if the OpMode is interrupted while waiting
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Mirror telemetry to FTC Dashboard so live data is visible on a laptop.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create Road Runner drive at origin heading 0°.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        // Construct the Bot.  false = use gamepad 1 for driving (single-driver mode).
        Bot bot = new Bot(hardwareMap, telemetry, drive, gamepad1, gamepad2, false);
        bot.teleopInit(); // reset all subsystems before the match starts

        waitForStart(); // pause here until the driver presses PLAY

        bot.teleopStart(); // position hardware for match start

        // Control loop: run every iteration until OpMode is stopped.
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick(); // execute one full control cycle
        }
    }
}