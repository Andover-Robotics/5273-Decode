package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ConcurrentLinkedQueue;

public class Indexer {
    private IndexerState state;
    private boolean intaking = true;

    private final IndexerState COLOR_SENSOR_POSITION = IndexerState.one;
    private ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };
    private final ColorSensorSystem colorSensor;

    public enum ArtifactColor {
        unknown,
        purple,
        green
    }

    public enum IndexerState
    {
        //i swear these names are temporary we'll do some color coding or sum
        one,
        two,
        three,
        oneAlt
    }
    private final SimpleServo indexerServo;
    private final Actuator actuator;

    // Color sensor timing stuff
    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelay;
    private final double msPerDegree = 0.6; // tune this
    private final double minWait = 100;
    private final double maxWait = 300;
    private double lastAngle = 0;
    public Indexer (HardwareMap hardwareMap)
    {
        state = IndexerState.one;
        indexerServo = new SimpleServo(hardwareMap, "index",0,360);
        colorSensor = new ColorSensorSystem(hardwareMap);
        actuator = new Actuator(hardwareMap);
    }

    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(nextState());
        }
    }

    public void startIntake()
    {
        setIntaking(true);
        //intakes from a closer container instead of the one currently at outtake
        moveTo(nextState());
    }

    public void startOuttake()
    {
        setIntaking(false);
        //moves to a closer state instead of turning the full 180 degrees
        moveTo(nextState());
    }

    public boolean getIntaking()
    {
        return intaking;

    }

    public ArtifactColor stateToColor(IndexerState colorState) {
        ArtifactColor color = ArtifactColor.unknown;
        int stateNum = stateToNum(colorState);
        if (stateNum == 4) stateNum = 1;
        stateNum -= 1;
        color = artifacts[stateNum];
        return color;
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
                artifacts = new ArtifactColor[]{artifacts[1],artifacts[2],artifacts[0]};
            } else {
                artifacts = new ArtifactColor[]{artifacts[2],artifacts[0],artifacts[1]};
            }
        }
        if (actuator.isActivated()) { //  && !getIntaking() not ever set to false yet
            int stateNum = stateToNum(state);
            if (stateNum == 4) stateNum = 1;
            stateNum -= 1;
            artifacts[stateNum] = ArtifactColor.unknown;
        }
    }
    public void moveToColor(ArtifactColor color) {
        ArtifactColor checkingColor = stateToColor(IndexerState.one);
        if (checkingColor == color) {moveTo(IndexerState.one); return;}
        checkingColor = stateToColor(IndexerState.three);
        if (checkingColor == color) {moveTo(IndexerState.three); return;}
        checkingColor = stateToColor(IndexerState.two);
        if (checkingColor == color) {moveTo(IndexerState.two);}
    }
    public void quickSpin()
    {
        switch(state)
        {
            case one:
                moveInOrder(new int[]{1,2,3});
                break;
            case two:
                moveInOrder(new int[]{2,3,1});
                break;
            case three:
                moveInOrder(new int[]{3,2,1});
                break;
            case oneAlt:
                moveInOrder(new int[]{1,3,2});
                break;
        }
    }

    // TODO Either low key get rid of this or implement wait(short thread.sleep should be fine?) there is no wait in between so color sensing doesn't work
    public void moveInOrder(int[] arr) {
        Thread moveThread = new Thread(() -> {
            for(int i : arr){
                moveTo(numToState(i));
            }
        });
        moveThread.start();
    }

    //for state 1: will turn to 0
    //for state oneAlt: stateToNum returns 4, so turns to 360
    public void moveTo(IndexerState newState)
    {
        shiftArtifacts(state, newState);
        double oldAngle = lastAngle;
        double newAngle;
        if(intaking)
        {
            newAngle = ((stateToNum(newState)-1)*120+60)%360; // intake angle calculation
            indexerServo.turnToAngle(newAngle);
        } else {
            newAngle = (stateToNum(newState) - 1) * 120; // outtake angle calculation
            indexerServo.turnToAngle(newAngle);
        }

        double angleDelta = Math.abs(newAngle - oldAngle);
        if (angleDelta > 180) angleDelta = 360 - angleDelta;  // shortest path

        double waitTime = Math.min(maxWait, Math.max(minWait, angleDelta * msPerDegree));

        scanTimer.reset();
        scanDelay = waitTime;
        scanPending = true;

        lastAngle = newAngle;

        state = newState;
    }

    public void updateColorScanning() {
        if (scanPending && scanTimer.milliseconds() >= scanDelay) {
            scanArtifact();
            scanPending = false;
        }
    }

    public IndexerState numToState(int num)
    {
        switch (num)
        {
            case 1:
                return closestZero();
            case 2:
                return IndexerState.two;
            case 3:
                return IndexerState.three;
        }
        return null;
    }

    //returns 4 for oneAlt (useful for turning functions as you can see in comments)
    public int stateToNum(IndexerState newState)
    {
        switch (newState)
        {
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

    //im not gonna bother explaining this because ur lowkey cooked if you dont understand this math
    public IndexerState nextState()
    {
        return numToState((stateToNum(state) % 3) + 1);
    }

    //returns the closest 0 state
    public IndexerState closestZero()
    {
        if(state == IndexerState.two) {
            return IndexerState.one;
        }
        if(state == IndexerState.three) {
            return IndexerState.oneAlt;
        }
        return state;
    }

    public IndexerState getState() {
        return state;
    }

    public boolean isBusy() {
        return scanPending;
    }
}
