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
            {0.2, 0.8}, // pickup para
            {0, 0.55}, // pickup perp
            {}, // pickup slanted
            {0.9, 0.2}, // score bucket
            {}
    };

    public enum WRIST_STATE {
        PICKUP_PARA, PIKCUP_VERT, BUCKET, CHAMBER, WALL
    }

    public static WRIST_STATE wristState;

    public void setWristState(WRIST_STATE ws) {
        this.wristState = ws;
    }
    public WRIST_STATE getWristState() {
        return this.wristState;
    }

    public void initialize(HardwareMap hardwareMap) {
        right = hardwareMap.get(Servo.class, "rServo");
        left = hardwareMap.get(Servo.class, "lServo");

        setWristState(WRIST_STATE.PICKUP_PARA);
        right.setPosition(positions[0][0]);
        left.setPosition(positions[0][1]);


    }

    public void update(Telemetry telemetry) {
        switch (wristState) {
            case PICKUP_PARA:
                right.setPosition(positions[0][0]);
                left.setPosition(positions[0][1]);
                break;
            case PIKCUP_VERT:
                right.setPosition(positions[1][0]);
                right.setPosition(positions[1][1]);
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