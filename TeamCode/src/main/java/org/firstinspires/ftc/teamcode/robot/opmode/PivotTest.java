package org.firstinspires.ftc.teamcode.robot.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;


@Config
@TeleOp
public class PivotTest extends OpMode {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double target = 0;

    private AbsoluteAnalogEncoder penc;
    private AnalogInput encV;
    private DcMotorEx pivot;

    private PIDController pid;



    @Override
    public void init() {
        pid = new PIDController(kP, kI, kD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        encV = hardwareMap.get(AnalogInput.class, "poop");
        penc = new AbsoluteAnalogEncoder(encV, 3.3);

        penc.zero(0.2323);
    }

    @Override
    public void loop() {
        pid.setPID(kP, kI, kD);

        double armPos = penc.getCurrentPosition();

        double pidP = pid.calculate(armPos, target);
        double ff = Math.cos(armPos) * kF;

        double power = pidP + ff;

        pivot.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);

        telemetry.addData("power", power);

        telemetry.update();
    }
}
