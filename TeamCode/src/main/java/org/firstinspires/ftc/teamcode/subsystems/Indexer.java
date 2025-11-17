package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

/*
Ways to outtake -
  1. actuator up then clockwise spin
  2. closest direction spin(counterclockwise or clockwise) then actuator up
this may require 2 different actuator up positions

TODO: At some point maybe implement some advanced things, like:
  - updating quickspin to always do it in the right order for obelisk pattern(must scan obelisk sometime before during teleop phase)
  - rotating to nearest free position and going to intake position in one,
  - directly getting to outtake pos for nearest color: starting Outtake, find closest of given color, indexing ball to outtake(so its on top of actuator)
    - consider maybe even a button that does the whole normal outtake and color outtake in one?(may require many sleeps)
  - other ideas,
  but maybe some only (too complicated is bad)
*/
@Config
public class Indexer {
    private IndexerState state;
    private boolean intaking = true;

    private final IndexerState COLOR_SENSOR_POSITION = IndexerState.one;
    private final IndexerState ACTUATOR_POSITION = IndexerState.two;
    public static int hz = 100;

    private ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };
    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl indexerServoControl;
    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelay;
    private final double minWait = 100;
    private final double maxWait = 300;
    public static double msPerDegree = 0.6;
    public static double targetAngle = 0;
    public static double offsetAngle = 105;
    public static double outtakeOffsetAngle = 5;
    private double lastAngle = offsetAngle;

    private Actuator actuator;
    private final AnalogInput indexerAnalogEncoder;

    // queue for single-threaded movement
    private final BlockingQueue<IndexerState> moveQueue = new LinkedBlockingQueue<>();
    private final UpdateThread updateThread = new UpdateThread();
    private Thread thread;
    private final Object artifactLock = new Object(); // prevent race conditions

    public Indexer(HardwareMap hardwareMap) {
        state = IndexerState.one;
        CRServo indexerServo = hardwareMap.get(CRServo.class, "index");
        indexerAnalogEncoder = hardwareMap.get(AnalogInput.class, "indexAnalog");
        actuator = new Actuator(hardwareMap);
        indexerServoControl = new CRServoPositionControl(indexerServo, indexerAnalogEncoder);
        colorSensor = new ColorSensorSystem(hardwareMap);

        thread = new Thread(updateThread);
        thread.start();
    }

    public enum ArtifactColor { unknown, purple, green }
    public enum IndexerState { one, two, three }

    public double getVoltageAnalog() {
        return indexerAnalogEncoder.getVoltage();
    }

    public void startIntake() {
        actuator.down();
        this.intaking = true;
        moveTo(state);
        queueMove(nextState());
    }
    public void startOuttake() {
        this.intaking = false;
        moveTo(state);
        queueMove(nextState());
    }
    public boolean getIntaking() { return intaking; }

    public ArtifactColor stateToColor(IndexerState colorState) {
        int stateNum = stateToNum(colorState);
        synchronized (artifactLock) {
            return artifacts[stateNum];
        }
    }

    public void moveToColor(ArtifactColor color) {
        if (stateToColor(IndexerState.one) == color) queueMove(IndexerState.one);
        else if (stateToColor(IndexerState.two) == color) queueMove(IndexerState.two);
        else if (stateToColor(IndexerState.three) == color) queueMove(IndexerState.three);
    }

    public IndexerState nextState() {
        return numToState((stateToNum(state) + 1) % 3); // due to 0 indexing this gets next state
    }

    public IndexerState getState() {
        return state;
    }

    public void queueMove(IndexerState newState) {
        moveQueue.add(newState);
    }
    public void queueMoves(IndexerState[] states) {
        for (IndexerState s : states) moveQueue.add(s);
    }

    private int stateToNum(IndexerState s) {
        return switch (s) {
            case one -> 0;
            case two -> 1;
            case three -> 2; };
    }
    private IndexerState numToState(int num) {
        return switch (num) {
            case 0 -> IndexerState.one;
            case 1 -> IndexerState.two;
            case 2 -> IndexerState.three;
            default -> null;
        };
    }

    // Calculate shortest path to target and move servo
    public void moveTo(IndexerState newState) {
        // Update artifact array
        shiftArtifacts(state, newState);

        double oldAngle = lastAngle;
        if (intaking) targetAngle = (stateToNum(newState) * 120 + 60) % 360;
        else targetAngle = stateToNum(newState) * 120;

        targetAngle = (targetAngle + offsetAngle) % 360;

        double clockwiseDelta = (targetAngle - oldAngle + 360) % 360;
        double counterDelta = (oldAngle - targetAngle + 360) % 360;
        if (counterDelta < clockwiseDelta && (!actuator.isActivated() || (actuator.isActivated() && intaking))) {
            targetAngle = (oldAngle - counterDelta + 360) % 360;
        }

        double angleDelta = Math.abs(((targetAngle - oldAngle + 540) % 360) - 180);

        double waitTime = Math.min(maxWait, Math.max(minWait, angleDelta * msPerDegree));
        scanTimer.reset();
        scanDelay = waitTime;
        scanPending = true;

        lastAngle = targetAngle;
        state = newState;
    }

    private void shiftArtifacts(IndexerState oldState, IndexerState newState) {
        int oldNum = stateToNum(oldState);
        int newNum = stateToNum(newState);

        // Calculate the shortest number of steps clockwise to reach new state
        int steps = (newNum - oldNum + 3) % 3; // ensures 0,1,2 steps

        synchronized (artifactLock) {
            ArtifactColor[] newArtifacts = new ArtifactColor[3];
            for (int i = 0; i < 3; i++) {
                newArtifacts[(i + steps) % 3] = artifacts[i];
            }
            artifacts = newArtifacts;

            // If actuator activated while outtaking, target and intermediary states should be unknown
            if (!intaking && actuator.isActivated()) {
                for (int i = 1; i <= steps; i++) {
                    int index = (oldNum + i) % 3;
                    artifacts[index] = ArtifactColor.unknown;
                }
            }
        }
    }

    private void scanArtifact() {
        ArtifactColor scannedColor = colorSensor.getColor();
        int stateNum = stateToNum(COLOR_SENSOR_POSITION);
        synchronized (artifactLock) {
            artifacts[stateNum] = scannedColor;
        }
    }

    private boolean isServoAtTarget() {
        double currentAngle = indexerServoControl.getAngle();
        double error = ((targetAngle - currentAngle + 540) % 360) - 180;
        return Math.abs(error) < CRServoPositionControl.angleDeadband;
    }

    private class UpdateThread implements Runnable {

        @Override

        public void run() {
            synchronized (artifactLock) {
                int delay = 1000 / hz;
                while (!Thread.currentThread().isInterrupted()) {
                    IndexerState next = null;
                    try {
                        next = moveQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    moveTo(next);

                    while (!Thread.currentThread().isInterrupted() && !isServoAtTarget()) {
                        indexerServoControl.moveToAngle(targetAngle);

                        if (scanPending && isServoAtTarget()) {
                            scanPending = false;
                            scanArtifact();
                        }
                        try {
                            Thread.sleep(delay);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }
            }
        }
    }

    public void actuatorUp() {
        actuator.up();

        // Only remove ball if outtaking and in correct pos (outtaked)
        if (!intaking && state == ACTUATOR_POSITION) {
            int actuatorIndex = stateToNum(ACTUATOR_POSITION);
            artifacts[actuatorIndex] = ArtifactColor.unknown;
        }
    }

    public void actuatorDown() {
        actuator.down();
    }

    public void stopThread() {
        if (thread != null && thread.isAlive()) thread.interrupt();
    }

}
