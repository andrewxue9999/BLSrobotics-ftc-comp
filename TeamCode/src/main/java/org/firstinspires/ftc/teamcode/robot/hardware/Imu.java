package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.IMU;

public class Imu {

    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    
    public void init(@NonNull HardwareMap hardwareMap, String name) {
        imu = hardwareMap.get(IMU.class, name);
    }
    public String getTelemetry(String name) {
        return String.format("Open %S", name);
    }
}
