package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

@Config
@TeleOp
public class Pivot extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    private static double offset = 1.2;

    public static int target = 0;

    private final double ticks_in_degree = 700/180.0;

    private DcMotorEx arm_motor;

    private AnalogInput poop;
    private AbsoluteAnalogEncoder penc;

    @Override
    public void init() {
        poop = hardwareMap.get(AnalogInput.class, "poop");
        penc = new AbsoluteAnalogEncoder(poop, 3.3);
        penc.zero(offset);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor0");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        double armPos = (Math.toDegrees(penc.getCurrentPosition()) * ticks_in_degree);
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
