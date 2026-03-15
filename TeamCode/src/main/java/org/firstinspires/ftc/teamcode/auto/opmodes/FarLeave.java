package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * FarLeave is a minimal "leave the starting zone" autonomous OpMode.
 *
 * <p>This is the simplest autonomous program: the robot simply drives straight
 * forward to the parking zone and stops. It is used when the field-side
 * ("far") starting position is used and there is no time to implement a more
 * complex autonomous routine, or as a guaranteed scoring fallback when more
 * complex autos are unreliable.
 *
 * <p>The robot starts at field position (0, 0) facing 0° (right), then
 * drives straight to ({@link #PARK_X}, {@link #PARK_Y}).  Both coordinates
 * are tunable at runtime via FTC Dashboard (the class is annotated with
 * {@link Config}).
 *
 * <p>Registered as {@code "Far Leave Auto"} in the {@code "Autonomous"} group.
 */
@Config
@Autonomous(name = "Far Leave Auto", group = "Autonomous")
public class FarLeave extends LinearOpMode {

    /**
     * Target X-coordinate (inches) for the parking position.
     * Positive X moves the robot forward (away from the driver station wall).
     * Default of 26 inches is enough to cross the field boundary line.
     */
    public static double PARK_X = 26;

    /**
     * Target Y-coordinate (inches) for the parking position.
     * Zero keeps the robot on the same lateral track as the starting position.
     * Adjust if the robot needs to dodge field elements while leaving.
     */
    public static double PARK_Y = 0;

    /**
     * Entry point called by the FTC runtime.
     * Builds the park trajectory, waits for start, then executes it.
     *
     * @throws InterruptedException (implicitly) if the OpMode is interrupted
     */
    @Override
    public void runOpMode() {
        // Define the starting pose: origin (0,0) facing 0° (pointing in +X direction).
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Construct all hardware subsystems via the Hardware container.
        // Hardware also builds the MecanumDrive and BotActions.
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);
        BotActions botActions = hardware.actions;     // action factory (unused here but available)
        MecanumDrive drive = hardware.mecanumDrive;  // Road Runner drive for trajectory building

        // The destination pose: (PARK_X, PARK_Y) still facing 0°.
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        // Build the single-segment trajectory: strafe straight from startPose to parkPose.
        // strafeTo drives along a straight line while maintaining the current robot heading.
        Action toPark = drive.actionBuilder(startPose)
                .strafeTo(
                        parkPose.position  // target x,y; heading is held constant at 0°
                )
                .build();

        waitForStart();                   // block here until the referee presses PLAY
        if (isStopRequested()) return;    // abort immediately if stop was pressed during init

        // Execute the parking trajectory.
        // ParallelAction with a single child is equivalent to running that child directly;
        // the wrapper is kept here for structural consistency with other autonomous OpModes.
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                toPark  // drive forward to the parking position
                        )
                )
        );
    }
}
