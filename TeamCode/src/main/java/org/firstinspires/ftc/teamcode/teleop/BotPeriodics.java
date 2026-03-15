package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Aimer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

/**
 * BotPeriodics is the base class for all teleop bot controllers.
 *
 * <p>It holds every hardware subsystem as a protected field and implements the
 * per-loop "periodic" handlers that must run every iteration regardless of the
 * current game state:
 * <ul>
 *   <li>Gamepad button edge detection (via FTCLib {@link GamepadEx})</li>
 *   <li>Intake control from both triggers on both gamepads</li>
 *   <li>Drive movement (robot-centric or field-centric)</li>
 *   <li>AprilTag heading lock / auto-aim update</li>
 *   <li>Alliance selection (goal target and LED colour)</li>
 *   <li>Telemetry output</li>
 *   <li>Subsystem periodic updates (indexer, outtake RPM, Road Runner pose)</li>
 *   <li>Action execution tick</li>
 * </ul>
 *
 * <p>Subclasses (e.g., {@link Bot}) override {@link #handleAllianceSelection()}
 * to add extra LED/haptics logic, and add game-state FSM code on top.
 *
 * <p>Annotated with {@link Config} so {@link #targetRPM} and any future static
 * fields are editable via FTC Dashboard.
 */
@Config
public class BotPeriodics {

    // -----------------------------------------------------------------------
    // Subsystems (protected so subclasses can access them directly)
    // -----------------------------------------------------------------------

    /** Conveyor / intake roller subsystem. */
    protected final Intake intake;

    /** Three-slot indexer drum subsystem. */
    protected final Indexer indexer;

    /** Actuator (launch ramp or ball-gate) subsystem. */
    protected final Actuator actuator;

    /** Flywheel shooter (outtake) subsystem. */
    protected final Outtake outtake;

    /** Mecanum drivetrain subsystem. */
    protected final Movement movement;

    /** Limelight AprilTag scanner subsystem. */
    protected final AprilTag aprilTag;

    /** Localization-based heading aimer. */
    protected final Aimer aimer;

    /** Road Runner MecanumDrive used for pose estimation and field-centric heading. */
    protected final MecanumDrive drive;

    // -----------------------------------------------------------------------
    // Gamepad wrappers (FTCLib – provide edge-detection on button presses)
    // -----------------------------------------------------------------------

    /** Driver 1 gamepad with edge-detection helpers. */
    protected final GamepadEx g1;

    /** Driver 2 (operator) gamepad with edge-detection helpers. */
    protected final GamepadEx g2;

    /** FTC telemetry object for writing Driver Station / Dashboard output. */
    protected final Telemetry telemetry;

    // -----------------------------------------------------------------------
    // Aiming / vision state
    // -----------------------------------------------------------------------

    /** Bearing turn-correction power from the last AprilTag/aimer update.
     *  Blended into the driver's rotation input to keep the robot aimed at the goal. */
    protected double bearingTurnCorrection = 0;

    /** Background Road Runner Action executor (one action at a time during teleop). */
    protected ActionHost actionHost;

    /** Whether field-centric drive mode is currently active. */
    protected boolean fieldCentric = false;

    /** When {@code true} the aimer runs every {@link #AIM_UPDATE_INTERVAL_MS} ms
     *  to maintain a continuous lock on the goal.  The robot's heading correction
     *  is applied to the drive command as long as this flag is set. */
    protected boolean continuousAprilTagLock = false;

    /** System time (ms) of the last aimer update tick.  Used to rate-limit calls to
     *  {@link Aimer#calculateLocalizedTurnPower()} to {@link #AIM_UPDATE_INTERVAL_MS}. */
    protected long lastAimUpdate = 0;

    /** The turn-correction value from the previous aimer update.  Held between
     *  updates so the motor command is not zero between update periods. */
    protected double lastTurnCorrection = 0.0;

    /** Current turn correction applied to the drive command this loop iteration. */
    protected double turnCorrection = 0.0;

    /** Human-readable alliance colour selected by the drivers ("Red" or "Blue").
     *  Shown in telemetry so the team can confirm the correct alliance is configured. */
    protected String colorGoalSelected = "";

    /** Whether the driver has requested an RPM spin-up/range calculation this tick. */
    protected boolean rangeRequested = false;

    /** Most recent aimer output: {@code {turnPower, range, bearing}}.
     *  Index 0 = turn power, Index 1 = range (inches), Index 2 = bearing (degrees). */
    protected double[] targetData = {0, 0, 0};

    /** When {@code true} the intake runs at slow speed continuously in the background
     *  (passive intake).  Toggled by the DPAD_DOWN button on gamepad 2. */
    protected boolean continuousIntake = true;

    // -----------------------------------------------------------------------
    // Shooter RPM configuration
    // -----------------------------------------------------------------------

    /**
     * Target RPM for the shooter flywheel.  Updated dynamically from the range
     * regression when aiming is active; can also be set manually via Dashboard.
     */
    public static double targetRPM = 3800;

    /**
     * Minimum interval (ms) between aimer recalculations.
     * At 20 ms = 50 Hz: fast enough for smooth correction without overloading
     * the control loop with expensive trigonometry every single tick.
     */
    protected static final long AIM_UPDATE_INTERVAL_MS = 20;

    // -----------------------------------------------------------------------
    // Drive mode
    // -----------------------------------------------------------------------

    /**
     * {@code true} = use gamepad 2 for drive inputs; {@code false} = gamepad 1.
     * Allows a "two-movement-mode" where the second driver also controls driving
     * (e.g., for practice or single-player competitions).
     */
    protected boolean twoMovementMode;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Constructs a BotPeriodics and initialises every hardware subsystem.
     *
     * @param hardwareMap  the robot's hardware map used to locate all devices
     * @param tele         FTC telemetry object; output goes to Driver Station and Dashboard
     * @param mecanumDrive pre-constructed Road Runner drive for pose estimation
     * @param gamepad1     the Driver 1 raw gamepad
     * @param gamepad2     the Driver 2 (operator) raw gamepad
     * @param useMovement  when {@code true} gamepad 2 also controls driving
     */
    public BotPeriodics(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean useMovement) {
        // Construct all hardware subsystems from the hardware map.
        intake   = new Intake(hardwareMap);
        indexer  = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM); // use RPM-mode for closed-loop control
        drive    = mecanumDrive;
        movement = new Movement(hardwareMap, drive);
        aprilTag = new AprilTag(hardwareMap, tele);
        aimer    = new Aimer(drive);

        // Wrap raw gamepads with FTCLib GamepadEx to get edge-detection helpers.
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        actionHost = new ActionHost(); // background action executor
        telemetry = tele;
        twoMovementMode = useMovement; // remember whether gamepad 2 also drives
    }

    // -----------------------------------------------------------------------
    // Main periodic handler
    // -----------------------------------------------------------------------

    /**
     * Runs all per-loop periodic tasks.  Must be called at the top of every
     * teleop iteration (before the subclass's state-machine switch).
     *
     * <p>Order of operations:
     * <ol>
     *   <li>Read button state from both gamepads (edge detection refresh).</li>
     *   <li>Toggle continuous intake on DPAD_DOWN (gamepad 2).</li>
     *   <li>Read both gamepads' triggers and decide intake direction/speed.</li>
     *   <li>Handle heading lock, drive movement, alliance selection, telemetry.</li>
     *   <li>Tick subsystem periodic updates: indexer, outtake, action host, pose estimator.</li>
     *   <li>If the outtake target RPM is nonzero, keep refreshing it at the current target.</li>
     *   <li>If aiming is requested, periodically recalculate the turn correction and target RPM.</li>
     * </ol>
     */
    protected void handlePeriodics()
    {
        // Refresh button edge-detection state for both gamepads.
        g1.readButtons();
        g2.readButtons();

        // DPAD_DOWN on gamepad 2 toggles passive (slow continuous) intake on/off.
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            continuousIntake = !continuousIntake;

        // Read trigger values from both gamepads.
        double leftTrigger  = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double leftTrigger2 = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger  = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double rightTrigger2 = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // True if either driver is pressing the left (run) or right (reverse) trigger.
        boolean leftDown  = leftTrigger  > TeleopConstants.Gamepad.TRIGGER_DEADZONE
                         || leftTrigger2 > TeleopConstants.Gamepad.TRIGGER_DEADZONE;
        boolean rightDown = rightTrigger  > TeleopConstants.Gamepad.TRIGGER_DEADZONE
                         || rightTrigger2 > TeleopConstants.Gamepad.TRIGGER_DEADZONE;

        // Check whether the indexer drum is aligned with the intake port.
        boolean inRange = indexer.isWithinTargetDegrees(5);

        // Intake control priority: left trigger > right trigger > passive slow > stop.
        if(leftDown){
            intake.run();            // full-speed forward intake
        }
        else if(rightDown) intake.runBackwards();   // reverse intake to unjam
        else if(continuousIntake) intake.runSlow(); // passive slow intake in background
        else intake.stop();                         // all triggers released and passive disabled

        // --- driver 1 periodic handlers (run every loop) ---
        handleAprilTagLock();    // update auto-aim heading lock state
        handleMovement();        // send joystick inputs to the drive motors
        handleAllianceSelection(); // check for alliance selection button presses

        handleTelemetry();       // push diagnostic data to Driver Station / Dashboard

        // Tick all subsystems that need a per-loop update.
        indexer.update();           // run indexer servo PD control and color sensing
        outtake.periodic();         // update flywheel RPM measurement and controller
        actionHost.update();        // advance any running teleop Action by one step
        drive.updatePoseEstimate(); // integrate odometry/IMU into the pose estimate

        // If the shooter is spinning, continuously refresh the RPM setpoint.
        if (outtake.getTargetRPM() > 0) {
            outtake.set(targetRPM * Bot.QUICKSPIN_OUTTAKE_RPM_SCALE); // hold RPM at the dialed-in target
        }

        // If range feedback is requested or continuous lock is active, periodically
        // recalculate the bearing turn correction and the range-dependent target RPM.
        if(rangeRequested || continuousAprilTagLock){
            long now = System.currentTimeMillis();

            // Rate-limit the aimer recalculation to AIM_UPDATE_INTERVAL_MS.
            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;
                targetData = aimer.calculateLocalizedTurnPower(); // {turnPower, range, bearing}
                lastTurnCorrection = targetData[0]; // save turn correction for the drive command
                targetRPM = outtake.getRegressionRPM(targetData[1]); // set RPM from range regression
                bearingTurnCorrection = targetData[2]; // save bearing for telemetry
            }
            turnCorrection = lastTurnCorrection; // apply the most recent turn correction to drive
        }
    }

    // -----------------------------------------------------------------------
    // Periodic handler helpers
    // -----------------------------------------------------------------------

    /**
     * Writes diagnostic data for all subsystems to the Driver Station
     * telemetry panel and FTC Dashboard.  Called once per loop iteration.
     */
    protected void handleTelemetry()
    {
        telemetry.addData("Field Centric", fieldCentric); // display current drive mode
        telemetry.addData("Indexer State", "%s -> %s",
                indexer.getState(), indexer.getState().next()); // current and next slot
        telemetry.addData("Indexer Voltages",
                "Target: %.3f , Actual: %.3f",
                indexer.getTargetVoltage(), indexer.getVoltage()); // servo position tracking
        telemetry.addData("Outtake RPM", outtake.getRPM());           // measured flywheel RPM
        telemetry.addData("Target RMP", outtake.getTargetRPM());      // commanded RPM setpoint
        telemetry.addData("Actuator up?", actuator.isActivated());     // actuator position flag
        telemetry.addData("Indexer Loaded?", indexer.isLoaded());      // drum load state
        telemetry.addData("April Lock", continuousAprilTagLock);       // heading lock active?
        telemetry.addData("Bot Range", targetData[1]);                 // distance to goal (inches)
        telemetry.addData("Alliance selected", colorGoalSelected);     // selected alliance colour
        telemetry.addData("Turn Correction:", turnCorrection);         // current yaw correction
        telemetry.addData("Intake power: ", intake.getPower());        // intake motor power
        telemetry.addData("Last Turn Correction", lastTurnCorrection); // previous yaw correction
        // Print per-slot colour and positional error for all three indexer slots.
        for (Indexer.IndexerState s : Indexer.IndexerState.values()) {
            telemetry.addData(
                    "Slot " + s.index,
                    "%s (err=%.1f°)",
                    indexer.getColorAt(s),          // color stored in this slot
                    indexer.debugSlotErrorDeg(s)    // how far off the slot is from its target angle
            );
        }
        telemetry.update(); // flush all data to the Driver Station display
    }

    /**
     * Checks for alliance selection button presses from driver 1 and updates
     * the Limelight pipeline, the aimer's target pose, and the telemetry label.
     *
     * <ul>
     *   <li>{@code BACK} – blue alliance</li>
     *   <li>{@code START} – red alliance</li>
     * </ul>
     *
     * <p>Subclasses may override this to add LED/haptic feedback on selection.
     */
    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aprilTag.setPipeline(0);  // pipeline 0 = blue goal AprilTag
            aimer.setBlueTarget();    // aim toward the blue alliance goal
            colorGoalSelected = "Blue"; // update telemetry label
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);  // pipeline 1 = red goal AprilTag
            aimer.setRedTarget();     // aim toward the red alliance goal
            colorGoalSelected = "Red"; // update telemetry label
        }
    }

    /**
     * Reads stick inputs from the active driver gamepad and forwards them to
     * the drive subsystem, applying the current turn correction from the aimer.
     *
     * <p>If {@link #twoMovementMode} is {@code true} the drive inputs come from
     * gamepad 2 (operator) instead of gamepad 1 (driver).  This is useful for
     * single-player practice sessions.
     */
    protected void handleMovement() {
        // Read joystick axes from the appropriate gamepad.
        double lx = g1.getLeftX();  // driver 1 left stick X (strafe)
        double ly = g1.getLeftY();  // driver 1 left stick Y (forward)
        double rx = g1.getRightX(); // driver 1 right stick X (rotate)

        if(twoMovementMode){
            // Override with gamepad 2 inputs in two-movement mode.
            lx = g2.getLeftX();
            ly = g2.getLeftY();
            rx = g2.getRightX();
        }

        // Send to the drive subsystem in the appropriate mode.
        if (fieldCentric) movement.teleopTickFieldCentric(lx, ly, rx, turnCorrection, true);
        else              movement.teleopTick(lx, ly, rx, turnCorrection); // robot-centric mode
    }

    /**
     * Handles buttons that toggle the continuous AprilTag heading lock and
     * trigger a localizer re-zero.
     *
     * <ul>
     *   <li>{@code A} (gamepad 1) – enable continuous heading lock, 2 rumble blips</li>
     *   <li>{@code B} (gamepad 1) – disable heading lock, 1 rumble blip</li>
     *   <li>{@code X} (gamepad 1) – re-zero the Road Runner localizer at the
     *       human player zone for the current alliance</li>
     * </ul>
     *
     * <p>When the lock is disabled {@link #turnCorrection} is set to 0 so the
     * robot doesn't keep spinning based on a stale correction value.
     */
    protected void handleAprilTagLock() {
        // A: enable continuous heading lock toward the goal.
        if (g1.wasJustPressed(GamepadKeys.Button.A)) {
            continuousAprilTagLock = true;
            g1.gamepad.rumbleBlips(2); // 2 blips = lock engaged
        }

        // B: disable continuous heading lock.
        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            continuousAprilTagLock = false;
            g1.gamepad.rumbleBlips(1); // 1 blip = lock released
        }

        // X: re-zero the localizer at the current alliance's human player zone.
        if (g1.wasJustPressed(GamepadKeys.Button.X)) {
            // Always relocalize regardless of which alliance is selected.
            if (colorGoalSelected.equals("Blue"))
                aimer.relocalize(); // re-zero for blue human player zone
            else if (colorGoalSelected.equals("Red"))
                aimer.relocalize(); // re-zero for red human player zone
            else {
                aimer.relocalize(); // default relocalize even if no alliance was explicitly set
            }
        }

        // If lock is off, clear any lingering turn correction from the last active period.
        if (!continuousAprilTagLock){
            turnCorrection = 0; // no heading correction when lock is disabled
        }
    }

    // -----------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------

    /**
     * Returns the current target RPM for the shooter flywheel.
     * Subclasses use this to set the outtake RPM; it is exposed here so the
     * Dashboard can override it and so range-based RPM updates propagate.
     *
     * @return the current shooter target RPM
     */
    protected double getTargetRPM() {
        return targetRPM; // static field: shared across all instances and editable via Dashboard
    }
}

