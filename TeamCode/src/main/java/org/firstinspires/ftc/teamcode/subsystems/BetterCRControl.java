package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class BetterCRControl {
    private final CRServo crServo;
    private final AnalogInput encoder;

    public static double kp = 0.45;
    public static double ki = 0.0;
    public static double kd = 0.04;

    public static double deadband = 1.5;    // degrees
    public static double minPower = 0.08;
    public static double holdPower = 0.05;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double filteredVoltage = 0;

    private ElapsedTime timer = new ElapsedTime();

    public BetterCRControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        timer.reset();
    }

    private double getFilteredVoltage() {
        filteredVoltage = 0.6 * filteredVoltage + 0.4 * encoder.getVoltage();
        return filteredVoltage;
    }

    private double voltageToAngle(double v) {
        return (v / 3.2) * 360.0;
    }

    private double wrapDegrees(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    public void moveToAngle(double target) {
        double current = voltageToAngle(getFilteredVoltage());
        double error = wrapDegrees(target - current);

        // Hold state behavior
        if (Math.abs(error) < deadband) {
            crServo.setPower(holdPower * Math.signum(lastError));
            lastError = error;
            return;
        }

        double dt = timer.seconds();
        timer.reset();
        if (dt < 0.0001) dt = 0.0001;

        integral += error * dt;
        integral = Math.max(-2, Math.min(2, integral));

        double derivative = (error - lastError) / dt;

        double output = kp * error + ki * integral + kd * derivative;

        // Minimum power to break stiction
        if (Math.abs(output) < minPower)
            output = minPower * Math.signum(output);

        output = Math.max(-1.0, Math.min(1.0, output));

        crServo.setPower(output);

        lastError = error;
    }
}
