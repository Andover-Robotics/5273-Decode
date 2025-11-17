/* 
 * Autonomous code
 * Teleop and action methods
 * Uses RoadRunner
 * Setup and integration of RR in miscRR/MecanumDrive.java
 */

package org.firstinspires.ftc.teamcode.auto.roadrunner;

// Road Runner imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;

// Hardware imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Other
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import java.util.ArrayList;


/* --useful info--
 * Naming scheme - far means the side with both goals and near is opposite of it
 *
 * Robot starts at 1 of 2 locations:
 *  1. touching goal or far wall and touching far launch zone (Far opmodes)
 *  2. touching near wall and touching near launch zone (Near opmodes)
 *  And they must be on your alliance's side
 *
 * tiles are 24 * 24 inches
 */

// We will make it for all 4 positions, just because allied teams may only have certain auto positions
// If solo full auto is undoable and alliance partner has auto, we shall see how we want to do dividing work with the other team.
// red is right side, blue is left side (perspective of facing obelisk)
// Might/probably will split up this file for easier navigation and organization
public class AutoPathNavigator {

  // Positions numbered from bottom(nearest) rows up, and furthest from your alliance's wall to closest
  private static final Vector2d[][] artifact_positions = new Vector2d[3][3];

    // All of these positions could be redundant except the last of each if indexer works quick enough
  public static void runOpModeRedFar(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    Telemetry telemetry = opMode.telemetry;
    BotActions botActions = new BotActions(hardwareMap, telemetry);

    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
    
    int x_val_pose_0 = -20;
    int x_val_pose_1 = -25;
    int x_val_pose_2 = -30;

    int y_val_col_0 = 48;
    int y_val_col_1 = 72;
    int y_val_col_2 = 96;

    Vector2d shootPos = new Vector2d(-10, 10);

    artifact_positions[2][0] = new Vector2d(x_val_pose_0, y_val_col_2);
    artifact_positions[2][1] = new Vector2d(x_val_pose_1, y_val_col_2);
    artifact_positions[2][2] = new Vector2d(x_val_pose_2, y_val_col_2);
    artifact_positions[1][0] = new Vector2d(x_val_pose_0, y_val_col_1);
    artifact_positions[1][1] = new Vector2d(x_val_pose_1, y_val_col_1);
    artifact_positions[1][2] = new Vector2d(x_val_pose_2, y_val_col_1);
    artifact_positions[0][0] = new Vector2d(x_val_pose_0, y_val_col_0);
    artifact_positions[0][1] = new Vector2d(x_val_pose_1, y_val_col_0);
    artifact_positions[0][2] = new Vector2d(x_val_pose_2, y_val_col_0);

    Vector2d parkPos = new Vector2d(0,100);

    Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(180)))
      .strafeToSplineHeading(new Vector2d(0, y_val_col_2), Math.toRadians(180))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[2][0], Math.toRadians(180))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[2][1], Math.toRadians(180))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[2][2], Math.toRadians(180))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(180))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_1), Math.toRadians(180))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[1][0], Math.toRadians(180))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[1][1], Math.toRadians(180))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[1][2], Math.toRadians(180))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(180))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_0), Math.toRadians(180))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[0][0], Math.toRadians(180))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[0][1], Math.toRadians(180))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(180)))
            .strafeToSplineHeading(artifact_positions[0][2], Math.toRadians(180))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(180))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)
      .strafeToSplineHeading(parkPos, Math.toRadians(180))
      .stopAndAdd(botActions.actionPark())
      .build();
    opMode.waitForStart();
    Actions.runBlocking(arcStrikeVelocity);
  }

  public static void runOpModeBlueFar(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    Telemetry telemetry = opMode.telemetry;
    BotActions botActions = new BotActions(hardwareMap, telemetry);

    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

    int x_val_pose_0 = 20;
    int x_val_pose_1 = 25;
    int x_val_pose_2 = 30;

    int y_val_col_0 = 48;
    int y_val_col_1 = 72;
    int y_val_col_2 = 96;

    Vector2d shootPos = new Vector2d(10, 10);

    artifact_positions[0][0] = new Vector2d(x_val_pose_0, y_val_col_2);
    artifact_positions[0][1] = new Vector2d(x_val_pose_1, y_val_col_2);
    artifact_positions[0][2] = new Vector2d(x_val_pose_2, y_val_col_2);
    artifact_positions[1][0] = new Vector2d(x_val_pose_0, y_val_col_1);
    artifact_positions[1][1] = new Vector2d(x_val_pose_1, y_val_col_1);
    artifact_positions[1][2] = new Vector2d(x_val_pose_2, y_val_col_1);
    artifact_positions[2][0] = new Vector2d(x_val_pose_0, y_val_col_0);
    artifact_positions[2][1] = new Vector2d(x_val_pose_1, y_val_col_0);
    artifact_positions[2][2] = new Vector2d(x_val_pose_2, y_val_col_0);

    Vector2d parkPos = new Vector2d(0,100);

    Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(0)))
      .strafeToSplineHeading(new Vector2d(0, y_val_col_2), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_1), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_0), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)
      .strafeToSplineHeading(parkPos, Math.toRadians(0))
      .stopAndAdd(botActions.actionPark())
      .build();
    opMode.waitForStart();
    Actions.runBlocking(arcStrikeVelocity);
  }

  public static void runOpModeRedNear(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    Telemetry telemetry = opMode.telemetry;
    BotActions botActions = new BotActions(hardwareMap, telemetry);

    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

    int x_val_pose_0 = 20;
    int x_val_pose_1 = 25;
    int x_val_pose_2 = 30;

    int y_val_col_0 = 24;
    int y_val_col_1 = 48;
    int y_val_col_2 = 72;

    Vector2d shootPos = new Vector2d(0, 0);

    artifact_positions[0][0] = new Vector2d(x_val_pose_0, y_val_col_2);
    artifact_positions[0][1] = new Vector2d(x_val_pose_1, y_val_col_2);
    artifact_positions[0][2] = new Vector2d(x_val_pose_2, y_val_col_2);
    artifact_positions[1][0] = new Vector2d(x_val_pose_0, y_val_col_1);
    artifact_positions[1][1] = new Vector2d(x_val_pose_1, y_val_col_1);
    artifact_positions[1][2] = new Vector2d(x_val_pose_2, y_val_col_1);
    artifact_positions[2][0] = new Vector2d(x_val_pose_0, y_val_col_0);
    artifact_positions[2][1] = new Vector2d(x_val_pose_1, y_val_col_0);
    artifact_positions[2][2] = new Vector2d(x_val_pose_2, y_val_col_0);

    Vector2d parkPos = new Vector2d(24,24);

    Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(0)))
      .strafeToSplineHeading(new Vector2d(0, y_val_col_2), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_1), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_0), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)
      .strafeToSplineHeading(parkPos, Math.toRadians(0))
      .stopAndAdd(botActions.actionPark())
      .build();
    opMode.waitForStart();
    Actions.runBlocking(arcStrikeVelocity);
  }

  public static void runOpModeBlueNear(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    Telemetry telemetry = opMode.telemetry;
    BotActions botActions = new BotActions(hardwareMap, telemetry);

    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

    int x_val_pose_0 = -20;
    int x_val_pose_1 = -25;
    int x_val_pose_2 = -30;

    int y_val_col_0 = 24;
    int y_val_col_1 = 48;
    int y_val_col_2 = 72;

    Vector2d shootPos = new Vector2d(0, 0);

    artifact_positions[0][0] = new Vector2d(x_val_pose_0, y_val_col_2);
    artifact_positions[0][1] = new Vector2d(x_val_pose_1, y_val_col_2);
    artifact_positions[0][2] = new Vector2d(x_val_pose_2, y_val_col_2);
    artifact_positions[1][0] = new Vector2d(x_val_pose_0, y_val_col_1);
    artifact_positions[1][1] = new Vector2d(x_val_pose_1, y_val_col_1);
    artifact_positions[1][2] = new Vector2d(x_val_pose_2, y_val_col_1);
    artifact_positions[2][0] = new Vector2d(x_val_pose_0, y_val_col_0);
    artifact_positions[2][1] = new Vector2d(x_val_pose_1, y_val_col_0);
    artifact_positions[2][2] = new Vector2d(x_val_pose_2, y_val_col_0);

    Vector2d parkPos = new Vector2d(-24,24);

    Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(180)))
      .strafeToSplineHeading(new Vector2d(0, y_val_col_2), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_2, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[2][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_1), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_1, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[1][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)

      .strafeToSplineHeading(new Vector2d(0, y_val_col_0), Math.toRadians(0))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][0], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][1], Math.toRadians(0))
            .build()
        ))
      .stopAndAdd(new ParallelAction(
          botActions.actionIntake(),
          mecanumDrive.actionBuilder(new Pose2d(0, y_val_col_0, Math.toRadians(0)))
            .strafeToSplineHeading(artifact_positions[0][2], Math.toRadians(0))
            .build()
        ))
      .waitSeconds(.5)
      .strafeToSplineHeading(shootPos, Math.toRadians(0))
      .waitSeconds(.5)
      .stopAndAdd(botActions.actionOuttake())
      .waitSeconds(.5)
      .strafeToSplineHeading(parkPos, Math.toRadians(0))
      .stopAndAdd(botActions.actionPark())
      .build();
    opMode.waitForStart();
    Actions.runBlocking(arcStrikeVelocity);
  }

    public static void runOpModeRedParkFar(LinearOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        Telemetry telemetry = opMode.telemetry;
        BotActions botActions = new BotActions(hardwareMap, telemetry);

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

        Vector2d parkPos = new Vector2d(0,100);

        Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(180)))
                .strafeToSplineHeading(parkPos, Math.toRadians(0))
                .stopAndAdd(botActions.actionPark())
                .build();
        opMode.waitForStart();
        Actions.runBlocking(arcStrikeVelocity);
    }

    public static void runOpModeBlueParkFar(LinearOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        Telemetry telemetry = opMode.telemetry;
        BotActions botActions = new BotActions(hardwareMap, telemetry);

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

        Vector2d parkPos = new Vector2d(0,100);

        Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(180)))
                .strafeToSplineHeading(parkPos, Math.toRadians(0))
                .stopAndAdd(botActions.actionPark())
                .build();
        opMode.waitForStart();
        Actions.runBlocking(arcStrikeVelocity);
    }

    public static void runOpModeRedParkNear(LinearOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        Telemetry telemetry = opMode.telemetry;
        BotActions botActions = new BotActions(hardwareMap, telemetry);

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

        Vector2d parkPos = new Vector2d(24,24);

        Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(180)))
                .strafeToSplineHeading(parkPos, Math.toRadians(0))
                .stopAndAdd(botActions.actionPark())
                .build();
        opMode.waitForStart();
        Actions.runBlocking(arcStrikeVelocity);
    }

    public static void runOpModeBlueParkNear(LinearOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        Telemetry telemetry = opMode.telemetry;
        BotActions botActions = new BotActions(hardwareMap, telemetry);

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

        Vector2d parkPos = new Vector2d(-24,24);

        Action arcStrikeVelocity = mecanumDrive.actionBuilder(new Pose2d(0 , 0 , Math.toRadians(180)))
                .strafeToSplineHeading(parkPos, Math.toRadians(0))
                .stopAndAdd(botActions.actionPark())
                .build();
        opMode.waitForStart();
        Actions.runBlocking(arcStrikeVelocity);
    }
}
