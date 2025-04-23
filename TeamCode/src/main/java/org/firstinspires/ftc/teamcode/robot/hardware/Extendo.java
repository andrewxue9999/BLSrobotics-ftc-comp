package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.opmode.Teleop;

@Config
public class Extendo {
    DcMotorEx extendo;

    PIDController pid;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    private static double power;
    private static final double MAX_POWER = 0.5;
    private static double current;

    public static EXTENDO_STATES state;


    public static int extendoPos = 0;
    public static int target = 0;

    public static int RETRACTED = 0;
    public static int BHIGH = 0;
    public static int BLOW = 0;
    public static int CHIGH = 0;
    public static int CLOW = 0;
    public static int PICKUP = 0;



    public enum EXTENDO_STATES {
        BHIGH, BLOW, CHIGH, CLOW, PICKUP, WALL, RETRACTED, RESETING
    }

    public void setExtendoState(EXTENDO_STATES s) {
        state = s;
    }
    public EXTENDO_STATES getExtendoState() { return state; }

    public void initialize(HardwareMap hardwareMap) {
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pid = new PIDController(kP, kI, kD);
        pid.setPID(kP, kI, kD);
        pid.setTolerance(10);


    }

    public void update(Telemetry telemetry) {
        pid.setPID(kP, kI, kD);
//        switch(state) {
//            case BHIGH:
//                target = BHIGH;
//                break;
//            case BLOW:
//                target = BLOW;
//                break;
//            case CHIGH:
//                target = CHIGH;
//                break;
//            case CLOW:
//                target = CLOW;
//                break;
//            case PICKUP:
//                target = PICKUP;
//                break;
//            case RETRACTED:
//                target = RETRACTED;
//                break;
//        }

        double ff = kF * Math.cos(Pivot.getPivotPos());

        extendoPos = extendo.getCurrentPosition();

        power = pid.calculate(extendoPos, target) + ff;

        if(Math.abs(power)>MAX_POWER) {
            power = MAX_POWER * Math.signum(power);
        }

        current = extendo.getCurrent(CurrentUnit.AMPS);

        int error = Math.abs(target - extendoPos);

        extendo.setPower(power);

        telemetry.addData("extendoTarget", target);
        telemetry.addData("extendopos", extendoPos);
        telemetry.addData("error", error);
        telemetry.addData("current", current);


    }


}
