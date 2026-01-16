package org.firstinspires.ftc.teamcode.subsystems;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * POWER mode  → set(x) sets power 0–1
 * RPM mode    → set(x) sets RPM target and uses PIDF control
 */

@Config
public class Outtake {

    public enum Mode {
        POWER,
        RPM
    }

    private final MotorEx shooter;
    private final MotorEx shooter2;

    // FTCLib PID controller (P, I, D only)
    private final PIDController controller;

    // Dashboard-tunable gains
    public static double p = 0.000267;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0002;   // 1 / maxrpm and then tuned

    // Mode + state
    public Mode mode;
    private double motorPower = 0.0;
    public static double targetRPM = 2800.0; // without seeing any tags
    private double currentRPM = 0.0;

    private final double TPR = 28.0;   // encoder ticks per rotation

    public static double spinupInRangeMinTime = 300; // ms
    public static double spinupMaxTime = 4000; // ms
    private long inRangeStartTime = -1;
    private long spinupStartTime = -1;
    public static double INTAKE_MIN_RPM = 3500.0;

    public Outtake(HardwareMap hardwareMap, Mode mode) {
        shooter = new MotorEx(hardwareMap, "outtake");
        shooter.setInverted(true);
        shooter2 = new MotorEx(hardwareMap, "outtake-2");
        shooter2.setInverted(false);


        this.mode = mode;
        controller = new PIDController(p, i, d);
    }

    public void stop() {
        shooter.stopMotor();
        shooter2.stopMotor();
        motorPower = 0.0;
        targetRPM = 0.0;
    }

    /** Unified setter */
    public void set(double x) {
        if (mode == Mode.POWER) {
            motorPower = clamp(x, 0.0, 1.0);
        } else { // RPM MODE
            if (x != targetRPM) {
                spinupStartTime = -1;
                inRangeStartTime = -1;
            }
            targetRPM = x;
        }
    }

    public double getRPM() { return currentRPM; }
    public double getTargetRPM() { return targetRPM; }
    public double getPower() { return motorPower; }

    public void periodic() {
        // Update current RPM from motor encoder
        currentRPM = shooter.getVelocity() / TPR * 60.0;

        if (mode == Mode.POWER) {
            // Open-loop mode
            shooter.set(motorPower);
            shooter2.set(motorPower);
            return;
        }

        // Update PID gains live from dashboard
        controller.setPID(p, i, d);

        // Compute PID term
        double pid = controller.calculate(currentRPM, targetRPM);

        // Compute feedforward term from earlier code
        double ff = targetRPM * f;

        // Combined PIDF output
        motorPower = pid + ff;

        // Constrain power
        motorPower = clamp(motorPower, -1.0, 1.0);

        shooter.set(motorPower);
        shooter2.set(motorPower);
    }

    public double getRegressionRPM(double range)
    {
        if (Double.isNaN(range) || range <= 0) {
            return INTAKE_MIN_RPM;
        }
        return 0.00211836 * Math.pow(range, 3) - 0.614769 * Math.pow(range, 2) + 65.69185 * range + 1508.69255;
    }

    // Within the range and has been in range for spinupInRangeMinTime
    public boolean inRange(double tolerance) {
        long currentTime = System.currentTimeMillis();
        if (spinupStartTime == -1)
        {
            spinupStartTime = currentTime;
        }

        boolean withinTolerance = Math.abs(currentRPM - targetRPM) <= tolerance;

        if (withinTolerance)
        {
            if (inRangeStartTime == -1)
                inRangeStartTime = currentTime;
        }
        else
        {
                inRangeStartTime = -1;
        }

        boolean inRangeLongEnough = inRangeStartTime >= 0 && (currentTime - inRangeStartTime) >= spinupInRangeMinTime;
        boolean spunPastMaxTime = (currentTime - spinupStartTime) >= spinupMaxTime;

        if (inRangeLongEnough || spunPastMaxTime)
        {
            spinupStartTime = -1;
            inRangeStartTime = -1;
            return true;
        }

        return false;
    }
}
