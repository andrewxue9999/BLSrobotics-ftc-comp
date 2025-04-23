package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Claw {
    private Servo clawServo;

    public static double target = 0.0;

    private static double current;

    public static CLAW_STATES state = CLAW_STATES.INIT;

    public final double OPEN = 1.0;
    public final double CLOSED = 0.1;
    public final double INIT = 1.0;



    public enum CLAW_STATES {
        CLOSED, OPEN, INIT
    }

    public void setClawState(CLAW_STATES c) {
        state = c;
    }
    public CLAW_STATES getClawState() {
        return state;
    }

    public void initialize(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(INIT);
    }

    public void update(Telemetry telemetry) {
        switch(state) {
            case INIT:
                target = INIT;
                break;
            case OPEN:
                target = OPEN;
                break;
            case CLOSED:
                target = CLOSED;
                break;
        }

        clawServo.setPosition(target);


        telemetry.addData("State", state);

    }

}

