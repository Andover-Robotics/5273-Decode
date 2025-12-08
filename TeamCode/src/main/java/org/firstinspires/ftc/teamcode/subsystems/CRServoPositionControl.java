package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl
{
    //objects
    private final CRServo crServo;
    private final AnalogInput encoder; // Analog input for position from 4th wire
    private ElapsedTime timer = new ElapsedTime();

    // tuning constants
    public static double kp = 0.41;
    public static double ki = 0.0;
    public static double kf = 0.01;
    public static double filterAlpha = 0.9;

    // general constants
    private static final double ticksPerRev = 3.3;
    private static final double degreesPerRev = 360.0;

    // usage variables
    private double integral = 0.0;
    private double filteredVoltage = 0;
    private double targetVoltage;

    public CRServoPositionControl(CRServo servo, AnalogInput encoder)
    {
        this.crServo = servo;
        this.encoder = encoder;
        timer.reset();
    }

    public void moveToAngle(double targetAngleDegrees)
    {
        targetVoltage = angleToVoltage(targetAngleDegrees);
        double currentVoltage = getFilteredVoltage();

        double error = targetVoltage - currentVoltage;

        // Shortest path wrap handling, with offset to avoid error spikes
        double wrapPoint = (ticksPerRev / 2) + (ticksPerRev / 4); // Add offset
        if (error > wrapPoint) { error -= ticksPerRev; }
        if (error < -wrapPoint) { error += ticksPerRev; }

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0.0001) deltaTime = 0.0001;

        integral += error * deltaTime;
        integral = Math.max(-2, Math.min(2, integral));

        double output = kp * error + ki * integral + kf * Math.signum(error);
        output = Math.max(-1.0, Math.min(1.0, output));
        crServo.setPower(output);
    }

    private double getFilteredVoltage()
    {
        filteredVoltage = (1 - filterAlpha) * filteredVoltage + filterAlpha * encoder.getVoltage();
        return filteredVoltage;
    }

    private double angleToVoltage(double angleDegrees)
    {
        angleDegrees = Math.max(0, Math.min(degreesPerRev, angleDegrees)); // Clamp
        return (angleDegrees / degreesPerRev) * ticksPerRev;
    }

    public double getTargetVoltage()
    {
        return targetVoltage;
    }

    public double getCurrentAngle()
    {
        return (getFilteredVoltage() / ticksPerRev) * degreesPerRev;
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