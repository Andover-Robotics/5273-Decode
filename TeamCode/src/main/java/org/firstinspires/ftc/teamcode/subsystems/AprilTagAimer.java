package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AprilTagAimer {
    private final IMU imu;
    private final DcMotor leftFront, rightFront, leftBack, rightBack;
    private final Outtake outtake;
    private double power;
    private final double kP = 0.01; // Full PID can be done later
    private Double targetAngle = null;

    public AprilTagAimer(HardwareMap hardwareMap, Movement movement, Outtake outtake) {
        leftFront = movement.getLeftFront();
        rightFront = movement.getRightFront();
        leftBack = movement.getLeftBack();
        rightBack = movement.getRightBack();
        imu = movement.getImu();
        this.outtake = outtake;
        power = outtake.getPower();
    }

    public AprilTagAimer(HardwareMap hardwareMap, Movement movement) {
        leftFront = movement.getLeftFront();
        rightFront = movement.getRightFront();
        leftBack = movement.getLeftBack();
        rightBack = movement.getRightBack();
        imu = movement.getImu();
        this.outtake = null;
    }

    private double angleWrapDegrees(double angle) {
        return (angle + 180) % 360 - 180;
    }

    public void startTurnToAprilTag(double bearing) {
        double currentYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        targetAngle = angleWrapDegrees(currentYaw + bearing);
    }

    public boolean updateTurn() {
        if (targetAngle == null) return true;

        double currentYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        double error = angleWrapDegrees(targetAngle - currentYaw);

        if (Math.abs(error) < 1) { // within 1 degree
            stopMotors();
            targetAngle = null;
            return true;
        }
        else {
            double power = kP * error;
            power = Math.max(-1, Math.min(1, power));

            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(power);
            rightBack.setPower(-power);
            return false;
        }
    }

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void setPower(double range, double elevation) {
    // It'll be guess and check i guess
    outtake.setPower(power);
    }

    private void setPowerPhysicsBased(double range, double elevation) {
    /*
    * Calculates and sets the appropriate power for a flywheel shooter
    * based on the desired horizontal range and elevation angle.
    *
    * This method uses a physics-based model to determine the required
    * launch velocity (vInit) to reach a target at a certain distance and height,
    * accounting for both gravitational and aerodynamic drag losses.
    *
    * PHYSICS MODEL
    *
    * The launch velocity is computed by rearranging the projectile motion formula:
    *
    *      y = h + R * tan(θ) - (g * R²) / (2 * v² * cos²(θ))
    *
    * Solving for v:
    *
    *      v = sqrt((g * R²) / (2 * cos²(θ) * (R * tan(θ) + h - y)))
    *
    * Where:
    *   - R  = horizontal range to target
    *   - θ  = launch angle in radians
    *   - h  = launch height (hood or shooter height)
    *   - y  = target height
    *   - g  = gravitational acceleration
    *
    * Drag is modeled as:
    *      Fdrag = (1/2) * Cd * ρ * A * v²
    *      Pdrag = Fdrag * v = (1/2) * Cd * ρ * A * v³
    *
    * Drag energy loss is approximated as:
    *      dragEnergy = Pdrag * flightTime
    *
    * The total energy required is:
    *      totalEnergy = kineticEnergy + dragEnergy
    *
    * Mechanical power is then:
    *      requiredPower = totalEnergy / launchDuration
    *
    * Final motor power is scaled by:
    *      powerFraction = requiredPower / maxMotorPower
    *
    * ASSUMPTIONS
    * - Ball is launched at a fixed elevation angle.
    * - Drag is constant over short trajectory (reasonable for short-range shots).
    * - Flywheel imparts energy over a short, fixed time window.
    * - No Magnus effect or backspin modeling.
    * - Perfect energy transfer (can be tuned with efficiency factor if needed).
    *
    * TUNABLE PARAMETERS
    * - Cd (drag coefficient)
    * - launchDuration (how quickly flywheel imparts energy)
    * - Pmax (max mechanical power output of motor)
    *
    */

    // Constants
    final double g = 9.80665;                   // gravitational acceleration (m/s^2)
    final double h = 0;                         // UNKNOWN launch height (m)
    final double y = 1.175;                     // target height (m) bottom opening lip is .9845, top is 1.3655
    final double rFlywheel = 0;                 // UNKNOWN flywheel radius (m)
    final double Cd = 0.6;                      // drag coefficient
    final double rho = 1.21;                    // air density (kg/m^3)
    final double ballRadius = 0.061341;         // 4.83-inch diameter ball
    final double A = Math.PI * ballRadius * ballRadius;  // cross-sectional area (m²)
    final double maxRPM = 0;                    // UNKNOWN max RPM of flywheel motor
    final double Pmax = 0;                      // UNKNOWN max mechanical power output of motor (W)
    final double ballMass = 0.15;               // UNKNOWN should measure ball mass (kg)
    final double launchDuration = 0.1;          // time flywheel imparts energy to ball (s)

    // Angle conversions
    double theta = Math.toRadians(elevation);
    double cosTheta = Math.cos(theta);
    double tanTheta = Math.tan(theta);
    double cosThetaSquared = cosTheta * cosTheta;

    // Check if physically reachable
    double denominator = 2 * cosThetaSquared * (range * tanTheta + h - y);
    if (denominator <= 0) {
        telemetry.addData("Invalid trajectory:",  "Denominator ≤ 0 (unreachable target)");
        telemetry.update();
        outtake.setPower(0);
        return;
    }

    // Required initial velocity
    double vInit = Math.sqrt((g * range * range) / denominator);

    // Drag Force and power
    double Fdrag = 0.5 * Cd * rho * A * vInit * vInit;  // drag force at launch
    double Pdrag = Fdrag * vInit;                      // instantaneous drag power loss

    // Approximate flight time
    double vHorizontal = vInit * cosTheta;
    double flightTime = range / vHorizontal;

    // Energy calcs
    double kineticEnergy = 0.5 * ballMass * vInit * vInit;       // energy needed to reach vInit
    double dragEnergy = Pdrag * flightTime;                      // approximate energy lost to drag
    double totalEnergyRequired = kineticEnergy + dragEnergy;     // total energy to launch

    // Needed mechanical power
    double requiredMechanicalPower = totalEnergyRequired / launchDuration;

    double powerFraction = requiredMechanicalPower / Pmax;
    powerFraction = Math.max(0, Math.min(1, powerFraction));

    outtake.setPower(powerFraction);
  }
}
