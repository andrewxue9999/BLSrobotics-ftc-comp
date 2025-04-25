package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.opmode.Teleop;

@Config
public class DifferentialWrist {
    public Servo right;
    public Servo left;

    public static double[][] positions = {
            {}, // pickup para
            {}, // pickup perp
            {}, // pickup slanted
            {}, // score bucket
            {}
    };

    public enum WRIST_STATE {
        PICKUP, BUCKET, CHAMBER, WALL
    }

    public static WRIST_STATE wristState;

    public void setWristState(WRIST_STATE ws) {
        this.wristState = ws;
    }
    public WRIST_STATE getWristState() {
        return this.wristState;
    }

    public void initialize(HardwareMap hardwareMap) {
        right = hardwareMap.get(Servo.class, "wright");
        left = hardwareMap.get(Servo.class, "wleft");


    }

    public void update(Telemetry telemetry) {
        switch (wristState) {
            case PICKUP:
                right.setPosition(positions[0][0]);
                left.setPosition(positions[0][1]);
                break;
            case BUCKET:
                right.setPosition(positions[3][0]);
                left.setPosition(positions[3][1]);
                break;
//            case CHAMBER:
//                right.setPosition(positions[0][0]);
//                left.setPosition(positions[0][1]);
//                break;
//            case WALL:
//                right.setPosition(positions[0][0]);
//                left.setPosition(positions[0][1]);
//                break;
        }
        

    }





}