package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder; // Analog input for position from 4th wire

    public static double kp = 0.41;
    public static double ki = 0.0;
    public static double kd = 0.0; // should 0
    public static double kf = 0.015; // overcome deadband
    public static double filterAlpha = 0.90;
    public static double angleDeadband = 1.67; // degrees
    private double integral = 0.0;
    private double lastError = 0.0;
    private Double filteredVoltage = null;
    public final double MAX_VOLTAGE;
    private ElapsedTime timer = new ElapsedTime();

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        MAX_VOLTAGE = encoder.getMaxVoltage();
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

    private double getAngle() {
        return (getFilteredVoltage() / MAX_VOLTAGE) * 360.0;
    }

    public void moveToAngle(double targetAngleDegrees) {

        double currentAngle = getAngle();
        double error = ((targetAngleDegrees - currentAngle + 540) % 360) - 180; // shortest path

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

        double output = kp * error + ki * integral + kd * derivative + kf * Math.signum(error);
        output = Math.max(-1.0, Math.min(1.0, output));

        crServo.setPower(output);
        lastError = error;
    }
}
