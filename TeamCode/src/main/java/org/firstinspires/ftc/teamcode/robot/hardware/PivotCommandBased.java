package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

public class PivotCommandBased extends SubsystemBase {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double target = 0.0;

    private double armPos;
    private double power;
    private AbsoluteAnalogEncoder penc;
    private AnalogInput encV;
    private DcMotorEx pivot;

    private PIDController pid;

    Telemetry telemetry;

    public PivotCommandBased (@NonNull HardwareMap hardwareMap, String pivotName, String analogInputName)
    {
        pid = new PIDController(kP, kI, kD);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pivot = hardwareMap.get(DcMotorEx.class, pivotName);
        encV = hardwareMap.get(AnalogInput.class, analogInputName);
        penc = new AbsoluteAnalogEncoder(encV, 3.3);

        penc.zero(0.2323);


    }

    public void periodic() {
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);

        telemetry.addData("power", power);

        telemetry.update();
    }
    public void goToHigh() {
        pid.setPID(kP, kI, kD);

        target = 1.1;

        pid.setSetPoint(target);

        armPos = penc.getCurrentPosition();

        double pidP = pid.calculate(armPos);
        double ff = Math.cos(armPos) * kF;

        double power = pidP + ff;

        pivot.setPower(power);

    }

    public boolean reached ()
    {
        return pid.atSetPoint();
    }

}
