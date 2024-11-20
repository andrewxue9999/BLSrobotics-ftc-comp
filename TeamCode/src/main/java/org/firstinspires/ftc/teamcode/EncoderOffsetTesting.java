package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Encoder Offset Tester")

public class EncoderOffsetTesting extends LinearOpMode {

    private CRServo axon;

    private AbsoluteAnalogEncoder efrontRight;
    private AbsoluteAnalogEncoder efrontLeft;
    private AbsoluteAnalogEncoder ebackLeft;
    private AbsoluteAnalogEncoder ebackRight;
    

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        axon = hardwareMap.get(CRServo.class, "sfrontRight");

        efrontRight =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontRight"), 3.3);
        efrontLeft =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontLeft"), 3.3);
        ebackLeft =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackLeft"), 3.3);
        ebackRight=  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackRight"), 3.3);

        axon.setPower(0.5);

        TimeUnit.MILLISECONDS.sleep(150);

        axon.setPower(0);

        waitForStart();



        while (opModeIsActive()) {

            double frontRightV = efrontRight.getCurrentPosition();
            double frontLeftV = efrontLeft.getCurrentPosition();
            double backLeftV = ebackLeft.getCurrentPosition();
            double backRightV = ebackRight.getCurrentPosition();


            telemetry.addData("You can", "start aligning the left and right wheels with a straightedge!");

            telemetry.addData("frontRight", frontRightV + "rad");
            telemetry.addData("frontLeft", frontLeftV + "rad");
            telemetry.addData("backLeft", backLeftV + "rad");
            telemetry.addData("backRight", backRightV + "rad");

            telemetry.update();
        }
    }
}
