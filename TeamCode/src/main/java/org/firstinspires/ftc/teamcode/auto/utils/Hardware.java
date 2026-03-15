package org.firstinspires.ftc.teamcode.auto.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Aimer;

/**
 * Hardware is a data class that centralises construction and storage of every
 * robot subsystem needed during autonomous OpModes.
 *
 * <p>Creating a single {@code Hardware} object in the OpMode's {@code runOpMode()}
 * method ensures that:
 * <ul>
 *   <li>All subsystems are initialised in one place – easy to audit and update.</li>
 *   <li>Subsystems are shared by reference between the OpMode and
 *       {@link BotActions} without duplication.</li>
 *   <li>Autonomous-specific configuration (e.g., disabling automatic indexer
 *       advancement) is applied consistently.</li>
 * </ul>
 *
 * <p>All fields are {@code public final} so they can be read directly without
 * getters; this keeps autonomous OpMode code concise.
 *
 * <p>Typical usage:
 * <pre>
 *   Hardware hw = new Hardware(hardwareMap, telemetry, this, startPose);
 *   hw.actions.initializeAuto(Indexer.IndexerState.zero).run(...);
 * </pre>
 */
public class Hardware {

    // -----------------------------------------------------------------------
    // Subsystem fields
    // -----------------------------------------------------------------------

    /** Conveyor/intake roller subsystem. */
    public final Intake intake;

    /** Three-slot indexer drum subsystem. */
    public final Indexer indexer;

    /** Flywheel shooter (outtake) subsystem, configured in RPM mode. */
    public final Outtake outtake;

    /** Actuator (launch ramp or ball-gate) subsystem. */
    public final Actuator actuator;

    /** Limelight AprilTag scanner subsystem. */
    public final AprilTag aprilTag;

    /** Localization-based heading aimer. */
    public final Aimer aprilAimer;

    /** High-level action factory that builds Road Runner Action sequences for
     *  autonomous routines (intake, outtake, scanning, etc.). */
    public final BotActions actions;

    /** Road Runner MecanumDrive used for trajectory execution and pose estimation. */
    public final MecanumDrive mecanumDrive;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Constructs all robot subsystems and applies autonomous-specific configuration.
     *
     * <p>Autonomous-specific settings applied to the indexer:
     * <ul>
     *   <li>{@code AUTO_ADVANCEMENT_MODE = DISABLED} – turns off the automatic
     *       slot-advance-on-artifact-detected logic; advancement is controlled
     *       explicitly by the autonomous sequence instead.</li>
     *   <li>{@code ENABLE_FULL_UNKNOWN_SCAN = false} – disables the expensive
     *       full-unknown colour re-scan; in auto the colours are known in advance.</li>
     *   <li>{@code SCAN_COLORS = false} – disables live colour sensor reads in
     *       auto to reduce loop time and avoid false positives.</li>
     * </ul>
     *
     * @param hardwareMap the robot's hardware map used to retrieve all devices
     * @param telemetry   FTC telemetry for the AprilTag subsystem debug output
     * @param opMode      the running LinearOpMode (passed to BotActions for stop checks)
     * @param startPose   the robot's field starting pose for Road Runner's localizer
     */
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, Pose2d startPose) {
        // Build Road Runner MecanumDrive at the provided starting pose.
        mecanumDrive = new MecanumDrive(
                hardwareMap,
                startPose     // initialise localizer with match starting position
        );

        // Construct each subsystem from the hardware map.
        intake   = new Intake(hardwareMap);
        indexer  = new Indexer(hardwareMap);

        // Autonomous-specific indexer configuration:
        indexer.AUTO_ADVANCEMENT_MODE = Indexer.AutoAdvancement.DISABLED; // explicit control
        indexer.ENABLE_FULL_UNKNOWN_SCAN = false; // skip re-scan (colours known in advance)
        indexer.SCAN_COLORS = false;              // disable live sensor reads to save loop time

        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM); // RPM closed-loop shooter
        actuator = new Actuator(hardwareMap);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new Aimer(mecanumDrive);

        // Build the action factory last so it can reference all subsystems above.
        actions = new BotActions(this, telemetry, opMode);
    }
}
