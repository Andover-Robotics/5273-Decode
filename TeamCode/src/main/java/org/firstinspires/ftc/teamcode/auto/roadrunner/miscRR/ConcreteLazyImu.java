package org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class ConcreteLazyImu implements LazyImu {
    private final HardwareMap hardwareMap;
    private final String name;
    private final RevHubOrientationOnRobot orientation;
    private IMU imu;

    public ConcreteLazyImu(HardwareMap hardwareMap, String name, RevHubOrientationOnRobot orientation) {
        this.hardwareMap = hardwareMap;
        this.name = name;
        this.orientation = orientation;
    }

    @NonNull
    @Override
    public IMU get() {
        return null;
    }

}
