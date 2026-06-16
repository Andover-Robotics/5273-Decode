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
    public static double p = 0.0015;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.000176;   // 1 / maxrpm and then tuned

    // Mode + state
    public Mode mode;
    private double motorPower = 0.0;
    public static double targetRPM = 2800.0; // without seeing any tags
    private double currentRPM = 0.0;

    private final double TPR = 28.0;   // encoder ticks per rotation

    public static double spinupInRangeMinTime = 150; // ms
    public static double spinupMaxTime = 2750; // ms
    private long inRangeStartTime = -1;
    private long spinupStartTime = -1;
    public static double INTAKE_MIN_RPM = 3200.0;
    public static double multiplierForTesting = 1;

    private static final double[][] REGRESSION_DATA = {
            {44.82, 2810},
            {46.11, 2820},
            {48.20, 2830},
            {50.51, 2840},
            {52.14, 2850},
            {54.14, 2865},
            {56.29, 2880},
            {58.17, 2900},
            {60.40, 2920},
            {62.11, 2955},
            {64.13, 2990},
            {66.16, 3025},
            {68.31, 3060},
            {70.02, 3095},
            {72.25, 3130},
            {74.29, 3170},
            {76.07, 3210},
            {80.07, 3280},
            {84.41, 3360},
            {88.37, 3440},
            {92.66, 3520},
            {96.16, 3610},
            {100.00, 3750}
    };

    private static final double[][] REGRESSION_DATA_REDUCED = {
            {44.82, 2810},
            {58.17, 2900},
            {70.02, 3095},
            {80.07, 3280},
            {100.00, 3750}
    };

    public Outtake(HardwareMap hardwareMap, Mode mode) {
        shooter = new MotorEx(hardwareMap, "outtake-1");
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

    //horners form for some reason
    private double cubicRegressionRPM(double range) {
        return range * (range * (range * -0.0024059 + 0.70381) - 45.76105) + 3660.57931;
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
        return cubicRegressionRPM(range) * multiplierForTesting;
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
