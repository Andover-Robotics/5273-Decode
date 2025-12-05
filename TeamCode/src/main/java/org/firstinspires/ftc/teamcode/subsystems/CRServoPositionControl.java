package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl
{
    // Hardware
    private final CRServo crServo;
    private final AnalogInput encoder;
    private final ElapsedTime timer = new ElapsedTime();

    // Tunables (Dashboard)
    public static double kp = 0.41;
    public static double ki = 0.00;
    public static double kf = 0.01;         // static feedforward
    public static double filterAlpha = 0.9; // EMA strength

    // Encoder constants
    private static final double ticksPerRev = 3.3;   // voltage range of encoder
    private static final double degreesPerRev = 360; // full circle

    // Target state
    private double targetVoltage = 0.0;

    // Internal integrator
    private double integral = 0.0;

    // Filter + unwrapping state
    private double filteredVoltage = 0.0;
    private double lastRaw = 0.0;
    private double offset = 0.0;

    public CRServoPositionControl(CRServo servo, AnalogInput encoder)
    {
        this.crServo = servo;
        this.encoder = encoder;
        timer.reset();
    }

    // public api sigma boy

    public void moveToAngle(double angleDeg)
    {
        // Clamp physical command to 0–360
        angleDeg = clamp(angleDeg, 0.0, degreesPerRev);

        // Convert angle to target voltage (wrapped)
        targetVoltage = angleToVoltage(angleDeg);

        // Get filtered + unwrapped continuous voltage feedback
        double currentVoltage = getFilteredVoltage();

        // Compute wrapped shortest-path error
        double error = targetVoltage - currentVoltage;

        while (error >  ticksPerRev / 2.0) error -= ticksPerRev;
        while (error < -ticksPerRev / 2.0) error += ticksPerRev;

        // dt
        double dt = timer.seconds();
        timer.reset();
        if (dt < 0.0001) dt = 0.0001;

        // Integrator
        integral += error * dt;
        integral = clamp(integral, -2, 2);

        // PIDF
        double output = kp * error + ki * integral + kf * Math.signum(error);
        output = clamp(output, -1.0, 1.0);

        crServo.setPower(output);
    }

    // Get angle from unwrapped + filtered voltage
    public double getCurrentAngle()
    {
        return (getFilteredVoltage() / ticksPerRev) * degreesPerRev;
    }

    public double getTargetVoltage()
    {
        return targetVoltage;
    }

    // eocoder procesing
    private double getFilteredVoltage()
    {
        double raw = encoder.getVoltage(); // wrapped 0 → 3.3
        double diff = raw - lastRaw;

        // Detect wraparound and adjust offset
        if (diff > ticksPerRev / 2.0) {
            // 0.1 to 3.2 type shift
            offset -= ticksPerRev;
        } else if (diff < -ticksPerRev / 2.0) {
            // 3.2 to 0.1 type shift
            offset += ticksPerRev;
        }

        lastRaw = raw;

        // Unwrapped = raw + offset
        double unwrapped = raw + offset;

        // Smooth EMA filter on continuous value
        filteredVoltage = filteredVoltage * (1 - filterAlpha) + filterAlpha * unwrapped;

        return filteredVoltage;
    }

    // utils

    private double angleToVoltage(double deg)
    {
        return (deg / degreesPerRev) * ticksPerRev;
    }

    private static double clamp(double v, double min, double max)
    {
        return Math.max(min, Math.min(max, v));
    }
}

/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder;

    private static boolean angleIsLocked = false;
    private static double lockedAngle;
    public static double kp = 0.341;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0167;
    public static double lockedkp = 0.341;
    public static double lockedki = 0.0;
    public static double lockedkd = 0.0;
    public static double lockedkf = 0.0167;
    public static double filterAlpha = 0.80;
    public static double angleDeadband = 1.67;


    // Dynamic speed parameters
    public static double minSpeed = 0.15;           // minimal power to overcome deadband
    public static double maxErrorForScaling = 90.0; // error threshold for full speed

    private double integral = 0.0;
    private double lastError = 0.0;
    private Double filteredVoltage = null;
    public final double MAX_VOLTAGE;
    private ElapsedTime timer = new ElapsedTime();
    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        this.MAX_VOLTAGE = encoder.getMaxVoltage();
        timer.reset();
    }

    private double getFilteredVoltage() {
        if (filteredVoltage == null) {
            filteredVoltage = encoder.getVoltage();  // initialize to first reading
        } else {
            filteredVoltage = (1 - filterAlpha) * filteredVoltage + filterAlpha * encoder.getVoltage();
        }
        return filteredVoltage;
    }

    double getAngle() {
        return (getFilteredVoltage() / MAX_VOLTAGE) * 360.0;
    }

    public void lockAngle(double targetAngleDegrees) {
        angleIsLocked = true;
        lockedAngle = targetAngleDegrees;
    }
    public void unlockAngle() {
        angleIsLocked = false;
    }

    public void moveToAngle(double targetAngleDegrees) {
        double tempkp = kp;
        double tempki = ki;
        double tempkd = kd;
        double tempkf = kf;
        if (angleIsLocked) {
            targetAngleDegrees = lockedAngle;
            tempkp = lockedkp;
            tempki = lockedki;
            tempkd = lockedkd;
            tempkf = lockedkf;
        }

        double currentAngle = getAngle();
        double error = ((targetAngleDegrees - currentAngle + 540) % 360) - 180;

        if (Math.abs(error) < angleDeadband) {
            crServo.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0.0001) deltaTime = 0.0001;

        integral += error * deltaTime;
        integral = Math.max(-2, Math.min(2, integral));

        double derivative = (error - lastError) / deltaTime;

        double output = tempkp * error + tempki * integral + tempkd * derivative + tempkf * Math.signum(error);

        // Dynamic speed scaling, speeds up if distance is further
        double distanceFactor = Math.min(Math.abs(error) / maxErrorForScaling, 1.0);
        double scaledPower = minSpeed + (1.0 - minSpeed) * distanceFactor;
        output = Math.signum(output) * Math.min(Math.abs(output), scaledPower);

        crServo.setPower(output);
        lastError = error;
    }
}
*/
