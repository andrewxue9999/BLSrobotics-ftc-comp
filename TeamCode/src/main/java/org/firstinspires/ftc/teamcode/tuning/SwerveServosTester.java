package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

@Config
@TeleOp(name = "ServosTester")
public class SwerveServosTester extends OpMode {

    public CRServo frontLeft;
    public CRServo frontRight;
    public CRServo backLeft;
    public CRServo backRight;

    AnalogInput vfrontLeft;
    AnalogInput vfrontRight;
    AnalogInput vbackLeft;
    AnalogInput vbackRight;

    AbsoluteAnalogEncoder efrontLeft;
    AbsoluteAnalogEncoder efrontRight;
    AbsoluteAnalogEncoder ebackLeft;
    AbsoluteAnalogEncoder ebackRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(CRServo.class, "sfrontLeft");
        frontRight = hardwareMap.get(CRServo.class, "sfrontRight");
        backLeft = hardwareMap.get(CRServo.class, "sbackLeft");
        backRight = hardwareMap.get(CRServo.class, "sbackRight");

        vfrontLeft = hardwareMap.get(AnalogInput.class, "efrontLeft");
        vfrontRight = hardwareMap.get(AnalogInput.class, "efrontRight");
        vbackLeft = hardwareMap.get(AnalogInput.class, "ebackLeft");
        vbackRight = hardwareMap.get(AnalogInput.class, "ebackRight");

        efrontLeft = new AbsoluteAnalogEncoder(vfrontLeft, 3.3);
        efrontRight = new AbsoluteAnalogEncoder(vfrontRight, 3.3);
        ebackLeft = new AbsoluteAnalogEncoder(vbackLeft, 3.3);
        ebackRight = new AbsoluteAnalogEncoder(vbackRight, 3.3);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) frontRight.setPower(0.04);
        else frontRight.setPower(0);
        if (gamepad1.dpad_left) frontLeft.setPower(0.04);
        else frontLeft.setPower(0);
        if (gamepad1.dpad_down) backLeft.setPower(0.04);
        else backLeft.setPower(0);
        if (gamepad1.dpad_right) backRight.setPower(0.04);
        else backRight.setPower(0);

        telemetry.addData("Front Left", efrontLeft.getCurrentPosition());
        telemetry.addData("Front Right", efrontRight.getCurrentPosition());
        telemetry.addData("Back Left", ebackLeft.getCurrentPosition());
        telemetry.addData("Back Right", ebackRight.getCurrentPosition());



        telemetry.update();
    }
}
