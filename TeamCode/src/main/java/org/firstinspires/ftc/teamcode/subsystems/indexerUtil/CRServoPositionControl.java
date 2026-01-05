package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class CRServoPositionControl {

    // general constants
    public static double maxVoltage = 3.26;
    public static double degreesPerRev = 360.0;

    // gains
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.07; //voerriden later

    public static double maxPower = 1.0;
    public static double stiffnessGain = 1.0; //1.0 is normal behavior
    public static double brakeZoneDeg = 20.0;

    // deadbands
    public static double deadbandDeg = 6.0;
    public static boolean rotateClockwise = true;

    // Preset gains
    public static double unloaded_kP = 0.004;
    public static double unloaded_kI = 0.0;
    public static double unloaded_kD = 0.0;
    public static double unloaded_kS = 0.08;

    public static double loaded_kP = 0.004;
    public static double loaded_kI = 0.00009;
    public static double loaded_kD = 0.0;
    public static double loaded_kS = 0.08;

    private boolean loaded = false;
    private boolean manualOverride = false; // when true, update() does nothing (open-loop control active)

    // hardware
    private final CRServo servo;
    private final AnalogInput encoder;

    // state
    private double lastWrappedDeg;
    private double continuousDeg;
    private double targetDeg;

    private double lastAngleDeg = 0.0;
    private long lastTimeNs = 0;

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;

        double initial = getWrappedAngle();
        lastWrappedDeg = initial;
        continuousDeg = initial;
        targetDeg = initial;

        lastAngleDeg = continuousDeg;
        lastTimeNs = System.nanoTime();
    }

    //mroe balls is more gains
    public void setLoaded(boolean hasBalls) {
        this.loaded = hasBalls;
        if (hasBalls) {
            kP = loaded_kP;
            kI = loaded_kI;
            kD = loaded_kD;
            kS = loaded_kS;
        } else {
            kP = unloaded_kP;
            kI = unloaded_kI;
            kD = unloaded_kD;
            kS = unloaded_kS;
        }
    }

    public boolean isLoaded() { return loaded; }

    public void setOpenLoopPower(double power) {
        manualOverride = true;
        servo.setPower(clamp(power, -1.0, 1.0));
    }

    public void clearOpenLoop() {
        manualOverride = false;
    }

    public void update() {
        if (manualOverride) return; // skip closed-loop when in override

        updateContinuousAngle();

        double error = targetDeg - continuousDeg;
        double absErr = Math.abs(error);

        if (absErr < deadbandDeg) {
            servo.setPower(0);
            lastAngleDeg = continuousDeg;
            lastTimeNs = System.nanoTime();
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastTimeNs) * 1e-9;
        if (dt <= 0) dt = 1e-3;

        double velocity = (continuousDeg - lastAngleDeg) / dt; // deg/s

        lastAngleDeg = continuousDeg;
        lastTimeNs = now;

        double output = kP * error * stiffnessGain - kD * velocity + kI * error;

        // static friction compensation toward target
        if (Math.signum(output) == Math.signum(error)) {
            double sign = Math.signum(output);
            output = sign * Math.max(Math.abs(output), kS);
        }

        output = clamp(output, -maxPower, maxPower);
        servo.setPower(output);
    }

    public void moveToAngle(double wrappedAngleDeg) {
        updateContinuousAngle();

        double currentWrapped = mod(continuousDeg, 360.0);

        double delta = wrappedAngleDeg - currentWrapped;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        // rotation direction
        if (rotateClockwise && delta < 0) delta += 360;
        if (!rotateClockwise && delta > 0) delta -= 360;

        targetDeg = continuousDeg + delta;
    }

    public void moveBy(double deltaDeg) {
        targetDeg += deltaDeg;
    }

    public void reset() {
        double wrapped = getWrappedAngle();
        lastWrappedDeg = wrapped;
        continuousDeg = wrapped;
        targetDeg = wrapped;
        servo.setPower(0);
    }

    //bencoder
    private void updateContinuousAngle() {
        double wrapped = getWrappedAngle();
        double delta = wrapped - lastWrappedDeg;

        // unwrap
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        continuousDeg += delta;
        lastWrappedDeg = wrapped;
    }

    private double getWrappedAngle() {
        double v = clamp(encoder.getVoltage(), 0.0, maxVoltage);
        return (v / maxVoltage) * degreesPerRev;
    }

    /* ================= UTIL ================= */
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
    private double mod(double v, double m) {
        double r = v % m;
        return (r < 0) ? r + m : r;
    }

    /* ================= DEBUG ================= */
    public double getCurrentAngle() { return continuousDeg; }
    public double getTargetAngle() { return targetDeg; }
    public double getTargetVoltage() {
        double wrappedDeg = targetDeg % degreesPerRev;
        if (wrappedDeg < 0) wrappedDeg += degreesPerRev;
        return (wrappedDeg / degreesPerRev) * maxVoltage;
    }
    public double getVoltage() { return encoder.getVoltage(); }
}
