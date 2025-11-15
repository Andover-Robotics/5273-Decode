package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Indexer {
    private IndexerState state;
    private boolean intaking = true;
    Thread updateThread;
    private final IndexerState COLOR_SENSOR_POSITION = IndexerState.one;
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
    private final double msPerDegree = 0.6;
    private final double minWait = 100;
    private final double maxWait = 300;
    private double lastAngle = 0;
    public static double targetAngle = 0;
    Actuator actuator;
    AnalogInput indexerAnalog;

    public Indexer(HardwareMap hardwareMap) {
        state = IndexerState.one;
        CRServo indexerServo = hardwareMap.get(CRServo.class, "index");
        indexerAnalog = hardwareMap.get(AnalogInput.class, "indexAnalog");
        actuator = new Actuator(hardwareMap);
        indexerServoControl = new CRServoPositionControl(indexerServo, indexerAnalog);
        colorSensor = new ColorSensorSystem(hardwareMap);
    }
    private void start()
    {
        updateThread = new Thread(new UpdateThread());
        updateThread.start();
    }

    public enum ArtifactColor {
        unknown,
        purple,
        green
    }

    public enum IndexerState {
        one,
        two,
        three,
        oneAlt
    }

    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(nextState());
        }
    }

    public void startIntake() {
        setIntaking(true);
        moveTo(nextState());
    }

    public void startOuttake() {
        setIntaking(false);
        moveTo(nextState());
    }

    public boolean getIntaking() {
        return intaking;
    }

    public ArtifactColor stateToColor(IndexerState colorState) {
        int stateNum = stateToNum(colorState);
        if (stateNum == 4) stateNum = 1;
        stateNum -= 1;
        return artifacts[stateNum];
    }

    public void scanArtifact() {
        ArtifactColor scannedColor = colorSensor.getColor();
        int stateNum = stateToNum(COLOR_SENSOR_POSITION);
        if (stateNum == 4) stateNum = 1;
        stateNum -= 1;
        artifacts[stateNum] = scannedColor;
    }

    public void shiftArtifacts(IndexerState oldState, IndexerState newState) {
        int oldNum = stateToNum(oldState) - 1;
        if (oldNum == 3) oldNum = 0;
        int newNum = stateToNum(newState) - 1;
        if (newNum == 3) newNum = 0;

        int difference = newNum - oldNum;
        for (int i = 0; i < Math.abs(difference); i++) {
            if (difference < 0) {
                artifacts = new ArtifactColor[]{artifacts[1], artifacts[2], artifacts[0]};
            } else {
                artifacts = new ArtifactColor[]{artifacts[2], artifacts[0], artifacts[1]};
            }
        }
        if (actuator.isActivated()) {
            int stateNum = stateToNum(state);
            if (stateNum == 4) stateNum = 1;
            stateNum -= 1;
            artifacts[stateNum] = ArtifactColor.unknown;
        }
    }

    public void moveToColor(ArtifactColor color) {
        if (stateToColor(IndexerState.one) == color) {
            moveTo(IndexerState.one);
            return;
        }
        if (stateToColor(IndexerState.three) == color) {
            moveTo(IndexerState.three);
            return;
        }
        if (stateToColor(IndexerState.two) == color) {
            moveTo(IndexerState.two);
        }
    }

    public void quickSpin() {
        switch (state) {
            case one:
                moveInOrder(new int[]{1, 2, 3});
                break;
            case two:
                moveInOrder(new int[]{2, 3, 1});
                break;
            case three:
                moveInOrder(new int[]{3, 2, 1});
                break;
            case oneAlt:
                moveInOrder(new int[]{1, 3, 2});
                break;
        }
    }

    public void moveInOrder(int[] arr) {
        Thread moveThread = new Thread(() -> {
            for (int i : arr) {
                moveTo(numToState(i));
            }
        });
        moveThread.start();
    }

    // Only sets targetAngle and updates state and timing
    public void moveTo(IndexerState newState) {
        shiftArtifacts(state, newState);
        double oldAngle = lastAngle;
        if (intaking) {
            targetAngle = ((stateToNum(newState) - 1) * 120 + 60) % 360;
        } else {
            targetAngle = (stateToNum(newState) - 1) * 120;
        }
        double angleDelta = Math.abs(targetAngle - oldAngle);
        if (angleDelta > 180) angleDelta = 360 - angleDelta;

        double waitTime = Math.min(maxWait, Math.max(minWait, angleDelta * msPerDegree));
        scanTimer.reset();
        scanDelay = waitTime;
        scanPending = true;

        lastAngle = targetAngle;
        state = newState;
    }

    // Call this repeatedly in OpMode loop
    public void update() {
        indexerServoControl.moveToAngle(targetAngle);

        // some stuff the Ai spit out seems right but idk
        if (scanPending && scanTimer.milliseconds() >= scanDelay) {
            scanArtifact();
            scanPending = false;
        }


    }

    public IndexerState numToState(int num) {
        switch (num) {
            case 1:
                return closestZero();
            case 2:
                return IndexerState.two;
            case 3:
                return IndexerState.three;
        }
        return null;
    }

    public int stateToNum(IndexerState newState) {
        switch (newState) {
            case one:
                return 1;
            case two:
                return 2;
            case three:
                return 3;
            case oneAlt:
                return 4;
        }
        return 0;
    }

    public IndexerState nextState() {
        return numToState((stateToNum(state) % 3) + 1);
    }

    public IndexerState closestZero() {
        if (state == IndexerState.two) {
            return IndexerState.one;
        }
        if (state == IndexerState.three) {
            return IndexerState.oneAlt;
        }
        return state;
    }

    public double getVoltageAnalog() {
        return indexerAnalog.getVoltage();
    }

    public IndexerState getState() {
        return state;
    }

    public boolean isBusy() {
        return scanPending;
    }
    private class UpdateThread implements Runnable
    {

        @Override
        public void run() {
            int delay = 1000 / hz;
            while(!Thread.currentThread().isInterrupted())
            {
                update();
                try {
                    Thread.sleep(delay); // 50 Hz
                } catch (InterruptedException e) {
                    break; // thread stops safely
                }
            }
        }
    }
}
