package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

@Config
@TeleOp(name = "Intake Transfer Test", group = "testing")
public class IntakeTransferTest extends LinearOpMode {

    private Intake intake;
    private Storage storage;
    private Outtake outtake;
    private IntakeTransferTestBot testBot;

    public static double INTAKE_HOLDING_POWER = 0.3;
    public static double TRANSFER_HOLDING_POWER = -0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap, intake);
        outtake = new Outtake(hardwareMap, Outtake.Mode.POWER);

        testBot = new IntakeTransferTestBot(intake, storage, outtake, gamepad1, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            testBot.update();
            telemetry.update();
        }

        intake.stop();
        storage.stopTransfer();
        outtake.stop();
    }
}

class IntakeTransferTestBot {
    private final Intake intake;
    private final Storage storage;
    private final Outtake outtake;
    private final com.arcrobotics.ftclib.gamepad.GamepadEx gamepad;
    private final org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    private boolean intakeHolding = false;
    private boolean transferHolding = false;

    public IntakeTransferTestBot(
            Intake intake,
            Storage storage,
            Outtake outtake,
            com.qualcomm.robotcore.hardware.Gamepad gamepad,
            org.firstinspires.ftc.robotcore.external.Telemetry telemetry
    ) {
        this.intake = intake;
        this.storage = storage;
        this.outtake = outtake;
        this.gamepad = new com.arcrobotics.ftclib.gamepad.GamepadEx(gamepad);
        this.telemetry = telemetry;
    }

    public void update() {
        gamepad.readButtons();

        // A button: Toggle intake holding at normal holding amount
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            intakeHolding = !intakeHolding;
            if (intakeHolding) {
                intake.setPower(IntakeTransferTest.INTAKE_HOLDING_POWER);
            } else {
                intake.stop();
            }
        }

        // B button: Toggle transfer holding at normal holding amount
        if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
            transferHolding = !transferHolding;
            if (transferHolding) {
                storage.runTransfer();
                // For holding, we could manually set a lower power if needed
            } else {
                storage.stopTransfer();
            }
        }

        // Y button: Full intake power
        if (gamepad.getButton(GamepadKeys.Button.Y)) {
            intake.run();
        }

        // X button: Full transfer power
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            storage.runTransfer();
        }

        // Right bumper: Run both at holding amounts
        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            intake.setPower(IntakeTransferTest.INTAKE_HOLDING_POWER);
            storage.runTransfer();
        }

        // Left bumper: Stop both
        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            intake.stop();
            storage.stopTransfer();
        }

        // Update telemetry with dashboard-compatible format
        telemetry.addData("=== Intake Transfer Test ===", "");
        telemetry.addData("Intake Holding", intakeHolding);
        telemetry.addData("Transfer Holding", transferHolding);
        telemetry.addData("", "");
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Transfer Current (amps)", storage.getCurrent());
        telemetry.addData("Storage Full", storage.isFull());
        telemetry.addData("", "");
        telemetry.addData("Controls:", "");
        telemetry.addData("A - Toggle Intake Holding", "");
        telemetry.addData("B - Toggle Transfer Holding", "");
        telemetry.addData("Y - Full Intake", "");
        telemetry.addData("X - Full Transfer", "");
        telemetry.addData("RB - Both Holding", "");
        telemetry.addData("LB - Stop Both", "");
    }
}