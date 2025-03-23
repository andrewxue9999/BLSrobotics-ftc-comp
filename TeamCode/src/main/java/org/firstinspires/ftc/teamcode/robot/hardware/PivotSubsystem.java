package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.opmode.CommandBased;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

public class PivotSubsystem extends SubsystemBase {

    private AbsoluteAnalogEncoder penc;
    private AnalogInput encV;
    private DcMotorEx pivot;
    public static double PENCOFFSET = 0.2323;
    Telemetry telemetry;


    public PivotSubsystem (@NonNull HardwareMap hardwareMap, String pivotName, String analogInputName, Telemetry tele)
    {
        pivot = hardwareMap.get(DcMotorEx.class, pivotName);
        encV = hardwareMap.get(AnalogInput.class, analogInputName);
        penc = new AbsoluteAnalogEncoder(encV, 3.3);
        telemetry = tele;

    }

    public double getPivotPos() {
        return penc.getCurrentPosition();
    }

    @Override
    public void periodic() {
        telemetry.addData("Pivot pos", getPivotPos());
    }

    public void movePivot(double power) {
        pivot.setPower(power);
    }

}
