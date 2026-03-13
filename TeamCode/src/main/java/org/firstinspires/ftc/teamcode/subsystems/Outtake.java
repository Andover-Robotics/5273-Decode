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
    public static double p = 0.000567;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0002;   // 1 / maxrpm and then tuned

    // Mode + state
    public Mode mode;
    private double motorPower = 0.0;
    public static double targetRPM = 2800.0; // without seeing any tags
    private double currentRPM = 0.0;

    private final double TPR = 28.0;   // encoder ticks per rotation

    public static double spinupInRangeMinTime = 200; // ms
    public static double spinupMaxTime = 2750; // ms
    private long inRangeStartTime = -1;
    private long spinupStartTime = -1;
    public static double INTAKE_MIN_RPM = 3500.0;

    private static final double[][] REGRESSION_DATA = {
            {46.4, 3400},
            {48.3, 3450},
            {50.6, 3500},
            {52.6, 3550},
            {54.6, 3575},
            {56.5, 3600},
            {58.4, 3630},
            {61.7, 3660},
            {62.4, 3685},
            {64.6, 3690},
            {66.4, 3700},
            {68.5, 3710},
            {70.5, 3720},
            {72.6, 3728},
            {74.2, 3745},
            {76.4, 3760},
            {78.7, 3775},
            {80.3, 3830},
            {82.5, 3900},
            {84.5, 3930},
            {86.4, 3985},
            {88.6, 4100},
            {92.4, 4200},
            {96.4, 4280},
            {100.6, 4370},
            {104.5, 4460},
            {108, 4540},
            {112.4, 4600},
            {116.6, 4700}
    };

    private static final double[][] REGRESSION_DATA_REDUCED = {
            {46.4, 3400},
            {61.7, 3660},
            {78.7, 3775},
            {88.6, 4100},
            {116.6, 4700}
    };

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
        } else {
            if (Math.abs(x - targetRPM) > 25) {
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

    private double quarticRegressionRPM(double range) {
        return range * (range * (range * (range * -0.000297337 + 0.0958661) - 11.09971) + 562.06918) - 6981.95351;
    }

    private double cubicRegressionRPM(double range) {
        return range * (range * (range * -0.000281754 + 0.228245) - 13.14333) + 3623.28132;
    }

    private double linearInterpolation(double range, double[][] data) {
        double sum = 0;
        for (int i = 0; i < data.length - 1; i++) {
            double min = i == 0 ? Double.MIN_VALUE : data[i][0];
            double max = i == data.length - 2 ? Double.MAX_VALUE : data[i + 1][0];
            if (range >= min && range < max) sum +=
                    (range - data[i][0]) / (data[i + 1][0] - data[i][0]) *
                            (data[i + 1][1] - data[i][1]) + data[i][1];
        }
        return sum;
    }

    private double linearInterpolationRegressionRPM(double range) {
        return linearInterpolation(range, REGRESSION_DATA);
    }

    private double linearInterpolationRegressionReducedRPM(double range) {
        return linearInterpolation(range, REGRESSION_DATA_REDUCED);
    }

    public double getRegressionRPM(double range)
    {
        if (Double.isNaN(range) || range <= 0) {
            return INTAKE_MIN_RPM;
        }
        // Just use one of the three functions above
        return linearInterpolationRegressionReducedRPM(range);
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
