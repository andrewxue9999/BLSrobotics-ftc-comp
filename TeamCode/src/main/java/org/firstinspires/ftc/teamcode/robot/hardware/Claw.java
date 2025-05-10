package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Claw {
    private Servo clawServo;
    private ColorRangeSensor colorSesnor;

    public static double target = 0.0;

    private static double current;

    public static CLAW_STATES state = CLAW_STATES.CLOSED;

    public final double OPEN = 0.0;
    public final double CLOSED = 0.45;
    public final double INIT = 1.0;

    public final double clawWithSample = 0.05;

    private double rThresh = 0;
    private double bThresh = 0;



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
        colorSesnor = hardwareMap.get(ColorRangeSensor.class, "Colorado");
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
        telemetry.addData("claw servo pos", clawServo.getPosition());


    }

    public void actuate() {
        if (getClawState() == CLAW_STATES.OPEN) {
            setClawState(CLAW_STATES.CLOSED);
        } else if (getClawState() == CLAW_STATES.CLOSED) {
            setClawState(CLAW_STATES.OPEN);
        }
    }

    public boolean checkPickedUp(String allianceColor) {
        boolean res = false;
        boolean c = 0.08 > Math.abs(clawServo.getPosition() - clawWithSample);

        switch (allianceColor) {
            case "red" :
                if (colorSesnor.red() > rThresh && c) {
                    res = true;
                }
                break;
            case "blue" :
                if (colorSesnor.blue() > bThresh && c) {
                    res = true;
                }
                break;
        }

        return res;
    }

}

