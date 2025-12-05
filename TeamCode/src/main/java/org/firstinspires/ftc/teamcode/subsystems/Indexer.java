package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Indexer {

    public enum ArtifactColor { unknown, purple, green }

    public enum IndexerState {
        zero(0),
        one(1),
        two(2);

        public final int index;

        IndexerState(int index) {
            this.index = index;
        }

        public IndexerState next() {
            return values()[(this.index + 1) % 3];
        }
    }

    // config
    public static final double DEADBAND = 1.67;
    public static double offsetAngle = 105;
    public static double outtakeOffsetAngle = 5;
    public static double targetAngle = 0;

    // scan timing
    private static final double msPerDegree = 0.6;
    private static final double minWait = 100;
    private static final double maxWait = 300;

    // objects
    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl indexerServoControl;
    private final AnalogInput indexerAnalog;
    private final Actuator actuator;

    // internal state
    private IndexerState state = IndexerState.zero;
    private boolean intaking = true;

    private ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };

    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelay;

    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");
        indexerAnalog = hardwareMap.get(AnalogInput.class, "indexAnalog");

        actuator = new Actuator(hardwareMap);
        indexerServoControl = new CRServoPositionControl(servo, indexerAnalog);
        colorSensor = new ColorSensorSystem(hardwareMap);
    }

    // getters
    public IndexerState getState() { return state; }
    public boolean getIntaking() { return intaking; }
    public boolean isBusy() { return scanPending; }
    public double getVoltageAnalog() { return indexerAnalog.getVoltage(); }
    public double getTargetVoltage() { return indexerServoControl.getTargetVoltage(); }

    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state);
        }
    }
    // artifact color helpers
    public ArtifactColor stateToColor(IndexerState s) {
        return artifacts[s.index];
    }

    public void scanArtifact() {
        artifacts[state.index] = colorSensor.getColor();
    }

    // movement
    public void moveToColor(ArtifactColor color) {
        if (artifacts[0] == color) moveTo(IndexerState.zero);
        else if (artifacts[1] == color) moveTo(IndexerState.one);
        else if (artifacts[2] == color) moveTo(IndexerState.two);
    }

    public void moveTo(IndexerState newState) {
        double actualAngle = indexerServoControl.getCurrentAngle();

        // Slot = 0, 1, 2
        int slot = newState.index;

        // 120 per position
        targetAngle = slot * 120;

        if (!intaking) targetAngle += outtakeOffsetAngle;

        targetAngle = (targetAngle + offsetAngle) % 360;

        double delta = Math.abs(targetAngle - actualAngle);
        if (delta > 180) delta = 360 - delta;
        //if (delta < DEADBAND) return;

        double wait = Math.min(maxWait, Math.max(minWait, delta * msPerDegree));
        scanTimer.reset();
        scanDelay = wait;
        scanPending = true;

        state = newState;
    }

    public void update() {
        indexerServoControl.moveToAngle(targetAngle);

        if (scanPending && scanTimer.milliseconds() >= scanDelay) {
            scanArtifact();
            scanPending = false;
        }
    }

    public IndexerState nextState() {
        return state.next();
    }
}
