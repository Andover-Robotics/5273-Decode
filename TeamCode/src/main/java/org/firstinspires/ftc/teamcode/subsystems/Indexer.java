package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Indexer {

    //TODO: Optimize the algorithms, it'll work as is though

    // 0, 120, 240, 360 are the possible angles
    // since 0 and 360 are the same ball, no matter where you are,
    // you can "quick spin" to shoot all 3 balls quickly


    private IndexerState state;

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
        state = IndexerState.one;
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
        for(int i : arr){
            moveTo(numToState(i));
        }
    }

    public void moveTo(IndexerState newState)
    {
        indexerServo.turnToAngle((stateToNum(newState) - 1) * 120);
        state = newState;
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
