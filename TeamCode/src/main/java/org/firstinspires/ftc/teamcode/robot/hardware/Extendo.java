package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.opmode.Teleop;

@Config
public class Extendo {
    DcMotorEx extendo;

    public EXTENDO_STATES extendoState;
    public static int target = 0;
    public static int extendoPos = 0;

    public static int RETRACTED = 0;
    public static int BHIGH = 0;
    public static int BLOW = 0;
    public static int CHIGH = 0;
    public static int CLOW = 0;
    public static int PICKUP = 0;



    public static EXTENDO_STATES state;

    public enum EXTENDO_STATES {
        BHIGH, BLOW, CHIGH, CLOW, PICKUP, RETRACTED
    }

    public void setExtendoState(EXTENDO_STATES s) {
        state = s;
    }

    public void initialize(HardwareMap hardwareMap) {
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update(Telemetry telemetry) {
        extendoPos = extendo.getCurrentPosition();
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

        extendo.setTargetPosition(target);

        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendo.setPower(0.2);

        telemetry.addData("extendopos", extendoPos);
        telemetry.addData("extendoTarget", target);


    }


}
