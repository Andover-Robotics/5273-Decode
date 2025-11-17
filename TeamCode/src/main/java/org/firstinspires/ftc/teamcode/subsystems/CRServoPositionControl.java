package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder; // Analog input for position from 4th wire

    private static boolean angleIsLocked = false;
    private static double lockedAngle;
    public static double kp = 0.41;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.01;
    public static double lockedkp = 0.41;
    public static double lockedki = 0.0;
    public static double lockedkd = 0.0;
    public static double lockedkf = 0.01;
    public static double filterAlpha = 0.9;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double filteredVoltage = 0;
    private ElapsedTime timer = new ElapsedTime();

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        timer.reset();
    }

    private double getFilteredVoltage() {
        filteredVoltage = (1 - filterAlpha) * filteredVoltage + filterAlpha * encoder.getVoltage();
        return filteredVoltage;
    }


    private double angleToVoltage(double angleDegrees) {
        angleDegrees = Math.max(0, Math.min(360, angleDegrees)); // Clamp
        return (angleDegrees / 360.0) * 3.3;
        // REV Through-Bore analog encoders output 0–3.3V, not 0–3.2V apparently but we measured 3.2 so we will try
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

        double targetVoltage = angleToVoltage(targetAngleDegrees);
        double currentVoltage = getFilteredVoltage();

        double error = targetVoltage - currentVoltage;

        // Shortest path wrap handling, for continuous rotation (optional)
        if (error > 1.65) { error -= 3.3; }
        if (error < -1.65) { error += 3.3; }

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0.0001) deltaTime = 0.0001;

        integral += error * deltaTime;
        integral = Math.max(-2, Math.min(2, integral));
        double derivative = (error - lastError) / deltaTime;

        double output = tempkp * error + tempki * integral + tempkd * derivative + tempkf * Math.signum(error);
        output = Math.max(-1.0, Math.min(1.0, output));
        crServo.setPower(output);
        lastError = error;
    }
}
