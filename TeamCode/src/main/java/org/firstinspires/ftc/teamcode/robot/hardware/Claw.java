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

    public static CLAW_STATES state = CLAW_STATES.CLOSED;

    public final double OPEN = 1.0;
    public final double CLOSED = 0.0;
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
    }

    public void update(Telemetry telemetry) {
        switch(state) {
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

    public void actuate() {
        if (getClawState() == CLAW_STATES.OPEN) {
            clawServo.setPosition(CLOSED);
        } else if (getClawState() == CLAW_STATES.INIT) {
            clawServo.setPosition(OPEN);
        } else if (getClawState() == CLAW_STATES.CLOSED) {
            clawServo.setPosition(OPEN);
        }
    }

}

