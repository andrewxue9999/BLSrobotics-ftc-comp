
package org.firstinspires.ftc.teamcode.robot.hardware;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

@Config
public class Pivot {
    private DcMotorEx pivot;
    private AbsoluteAnalogEncoder penc;
    private AnalogInput analoginput;
    private static double EOFFSET = 6.28-0.6;

    PIDController pid;
    public static double kP = 1.4;
    public static double kI = 0.0;
    public static double kD = 0.07;
    public static double kF = 0.06;

    private static double power;
    private static final double MAX_POWER = 1;
    private static double current;

    public static PIVOT_STATES state = PIVOT_STATES.PICKUP;
    private static double pivotPos;
    public static double target;

    public final double HUNTING = 0.7;
    public final double PICKUP = 0.5;
    public final double HIGH = 2.2;
    public final double INIT = 1.3;
    public final double POST_PICKUP = 0.9;



    public enum PIVOT_STATES {
        PICKUP, SCORING, INIT, WALL, HUNTING, POST_PICKUP

    }
    public void setPivotState(PIVOT_STATES p) {
        state = p;
    }

    public PIVOT_STATES getPivotState() {
        return state;
    }

    public static double getPivotPos() { return pivotPos-0.7; }

    public void initialize(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        analoginput = hardwareMap.get(AnalogInput.class, "poop");
        penc = new AbsoluteAnalogEncoder(analoginput, 3.3);
        penc.zero(EOFFSET);

        pid = new PIDController(kP, kI, kD);
        pid.setPID(kP, kI, kD);
        pid.setTolerance(0.07);


        target = INIT;
        double power;
        double ff = kF * Math.cos(penc.getCurrentPosition()-0.7);
        pid.setSetPoint(target);
        pivotPos = penc.getCurrentPosition();
        while(!pid.atSetPoint()) {
            pivotPos = penc.getCurrentPosition();
            power = pid.calculate(pivotPos) + ff;
            pivot.setPower(power);
        }



    }

    public void update(Telemetry telemetry) {
        pid.setPID(kP, kI, kD);

        switch(state){
            case HUNTING:
                target = HUNTING;
                break;
            case PICKUP:
                target = PICKUP;
                break;
            case SCORING:
                target = HIGH;
                break;
            case INIT:
                target = INIT;
                break;
            case POST_PICKUP:
                target = POST_PICKUP;
                break;
        }

        double ff = kF * Math.cos(penc.getCurrentPosition()-0.7);

        pivotPos = penc.getCurrentPosition();

        power = pid.calculate(pivotPos, target) + ff;

        if(Math.abs(power)>MAX_POWER) {
            power = MAX_POWER * Math.signum(power);
        }
        current = pivot.getCurrent(CurrentUnit.AMPS);

        double error = Math.abs(target - pivotPos);

//        if (error < 0.3) {
//            power *= 0.4;
//        }

        pivot.setPower(power);

        telemetry.addData("target", target);
        telemetry.addData("pos", pivotPos);
        telemetry.addData("power", power);
        telemetry.addData("error", error);
        telemetry.addData("current", current);


    }

    public void setRawPower(double power) {
        pivot.setPower(power);
    }



}
