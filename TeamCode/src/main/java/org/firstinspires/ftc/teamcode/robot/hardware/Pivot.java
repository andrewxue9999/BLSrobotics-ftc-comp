// gear ratios
// calculations for limits to not exceed 42 inch horizontal limit

package org.firstinspires.ftc.teamcode.robot.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

@Config
public class Pivot {
    private DcMotorEx pivot;
    private AbsoluteAnalogEncoder penc;
    private AnalogInput analoginput;
    private static double EOFFSET = 0.0;

    PIDFController pid;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    private static double power;
    private static final double MAX_POWER = 0.5;
    private static double current;

    public static PIVOT_STATES state;
    private static double pivotPos;
    public static double target;

    public final double LOW = 1.5;
    public final double HIGH = 2.6;



    public enum PIVOT_STATES {
        PICKUP, SCORING

    }

    public void setPivotState(PIVOT_STATES p) {
        state = p;
    }

    public void initialize(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        analoginput = hardwareMap.get(AnalogInput.class, "poop");

        penc = new AbsoluteAnalogEncoder(analoginput, 3.3);
//        penc.zero(EOFFSET);

        pid = new PIDFController(kP, kI, kD, kF);


    }

    public void update(Telemetry telemetry) {
        pid.setPIDF(kP, kI, kD, kF);

        pivotPos = penc.getCurrentPosition();

        pid.setTolerance(0.1);
        power = pid.calculate(target, pivotPos);

        if(Math.abs(power)>MAX_POWER) {
            power = MAX_POWER * Math.signum(power);
        }

        pivot.setPower(power);

        telemetry.addData("target", target);
        telemetry.addData("pos", pivotPos);
        telemetry.addData("power", power);
        telemetry.addData("current", current);


    }



}
