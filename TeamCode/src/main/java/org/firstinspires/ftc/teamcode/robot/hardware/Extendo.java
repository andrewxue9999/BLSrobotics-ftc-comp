package org.firstinspires.ftc.teamcode.robot.hardware;

import static java.sql.Types.NULL;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Extendo {
    DcMotorEx extendo;

    PIDController pid;
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.08;

    private static double power;
    private static final double MAX_POWER = 1.0;
    private static double current;
    private static double maxCurrent;
    private static final double CURRENT_THRESHOLD = 4.4;

    public static EXTENDO_STATES state = EXTENDO_STATES.INIT;


    public static int extendoPos = 0;
    public static int target = 0;   

    public static int RETRACTED = 0;
    public static int BHIGH = 520;
    public static int BLOW = 0;
    public static int CHIGH = 0;
    public static int CLOW = 0;
    public static int PICKUP = 150;



    public enum EXTENDO_STATES {
        BHIGH, BLOW, CHIGH, CLOW, PICKUP, WALL, RETRACTED, RESETTING, INIT
    }

    public void setExtendoState(EXTENDO_STATES s) {
        state = s;
    }
    public EXTENDO_STATES getExtendoState() { return state; }

    public void initialize(HardwareMap hardwareMap) {
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        pid = new PIDController(kP, kI, kD);
        pid.setPID(kP, kI, kD);
        pid.setTolerance(10);


    }

    public void update(Telemetry telemetry) {
        pid.setPID(kP, kI, kD);
        switch(state) {
            case BHIGH:
                target = BHIGH;
                break;
            case BLOW:
                target = BLOW;
                break;
            case CHIGH:
                target = CHIGH;
                break;
            case CLOW:
                target = CLOW;
                break;
            case PICKUP:
                target = PICKUP;
                break;
            case RETRACTED:
                target = RETRACTED;
                break;
            case RESETTING:
                reset();
                break;
            case INIT:
                target = RETRACTED;
                break;
        }

        extendo.setTargetPosition(target);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(0.75);


        extendoPos = extendo.getCurrentPosition();

        double ff = kF * Math.cos(Pivot.getPivotPos());
        double PIDpower = pid.calculate(extendoPos, target);

        power = PIDpower + ff;

        if(Math.abs(power)>MAX_POWER) {
            power = MAX_POWER * Math.signum(power);
        }

        current = extendo.getCurrent(CurrentUnit.AMPS);

        int error = Math.abs(target - extendoPos);

        extendo.setPower(power);

        if(current>maxCurrent) {
            maxCurrent = current;
        }



        telemetry.addData("extendo PIDPower", PIDpower);
        telemetry.addData("extendo FF", ff);
        telemetry.addData("extendo real power", power);
        telemetry.addData("error", error);
        telemetry.addData("extendoTarget", target);
        telemetry.addData("extendopos", extendoPos);

        telemetry.addData("EXTENDO current", current);
        telemetry.addData("Max Current", maxCurrent);



    }


    public void reset() {
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendo.setPower(-0.7);

        extendo.setCurrentAlert(CURRENT_THRESHOLD, CurrentUnit.AMPS);

        if(extendo.isOverCurrent()) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
