package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.ConcurrentLinkedQueue;

public class Indexer {

    //TODO: Optimize the algorithms, it'll work as is though

    // 0, 120, 240, 360 are the possible angles
    // since 0 and 360 are the same ball, no matter where you are,
    // you can "quick spin" to shoot all 3 balls quickly


    private final int ANGLE_ONE = 0;
    private final int ANGLE_TWO = 120;
    private final int ANGLE_THREE = 240;
    private final int ANGLE_ALT = 360;
    private final IndexerState COLOR_SENSOR_POSITION = IndexerState.one;
    private ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };
    private final ColorSensorSystem colorSensor;

    private IndexerState state = IndexerState.one;

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

    public Indexer (HardwareMap hardwareMap)
    {
        indexerServo = new SimpleServo(hardwareMap, "index",0,360);
        colorSensor = new ColorSensorSystem(hardwareMap);
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

    public void moveInOrder(int[] arr) {
        Thread moveThread = new Thread(() -> {
            for(int i : arr){
                moveTo(numToState(i));
            }
        });
        moveThread.start();
    }

    public void moveTo(IndexerState newState)
    {
        shiftArtifacts(state, newState);
        state = newState;
        double newAngle = stateToAngle(newState);
        indexerServo.turnToAngle(newAngle);
        while (indexerServo.getAngle() > newAngle - 5 && indexerServo.getAngle() < newAngle + 5) {
            try {
                // Simulate the blocking operation of the servo
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        scanArtifact();
    }
    public double stateToAngle(IndexerState newState) {
        return (stateToNum(newState) - 1) * 120;
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

    public IndexerState nextState()
    {
        return numToState((stateToNum(state) % 3) + 1);
    }

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
}
