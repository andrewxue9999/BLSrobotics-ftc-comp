package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Encoder Offset Tester")
public class EncoderOffsetTesting extends LinearOpMode {



    private CRServo axon;

    private AbsoluteAnalogEncoder efrontRight;
    private AbsoluteAnalogEncoder efrontLeft;
    private AbsoluteAnalogEncoder ebackLeft;
    private AbsoluteAnalogEncoder ebackRight;
    private AbsoluteAnalogEncoder pivotEncoder;

    double newFrontRightRad;
    double newFrontRightDeg;
    double newFrontLeftRad;
    double newFrontLeftDeg;
    double newBackLeftRad;
    double newBackLeftDeg;
    double newBackRightRad;
    double newBackRightDeg;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        axon = hardwareMap.get(CRServo.class, "sfrontRight");


        axon.setPower(0.01);

        TimeUnit.MILLISECONDS.sleep(150);

        axon.setPower(0);

        efrontRight =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontRight"), 3.3);
        efrontLeft =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontLeft"), 3.3);
        ebackLeft =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackLeft"), 3.3);
        ebackRight=  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackRight"), 3.3);

        pivotEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "poop"), 3.3);
        pivotEncoder.zero(0.3);

        waitForStart();



        while (opModeIsActive()) {

            double pivot = ((pivotEncoder.getVoltage()) / 3.3) * 2 * Math.PI;
            pivotEncoder.setInverted(true);

            newFrontRightRad = ((efrontRight.getVoltage())/3.3) * 2 * Math.PI;
            newFrontRightDeg = ((efrontRight.getVoltage())/3.3) * 360;

            newFrontLeftRad = ((efrontLeft.getVoltage())/3.3) * 2 * Math.PI;
            newFrontLeftDeg = ((efrontLeft.getVoltage())/3.3) * 360;

            newBackLeftRad = ((ebackLeft.getVoltage())/3.3) * 2 * Math.PI;
            newBackLeftDeg = ((ebackLeft.getVoltage())/3.3) * 360;

            newBackRightRad = ((ebackRight.getVoltage())/3.3) * 2 * Math.PI;
            newBackRightDeg = ((ebackRight.getVoltage())/3.3) * 360;


            double frontRightV = efrontRight.getCurrentPosition();
            double frontLeftV = efrontLeft.getCurrentPosition();
            double backLeftV = ebackLeft.getCurrentPosition();
            double backRightV = ebackRight.getCurrentPosition();


            telemetry.addData("You can", "start aligning the left and right wheels with a straightedge!");

            telemetry.addData("newFrontRightRad", newFrontRightRad);
            telemetry.addData("newFrontRightDeg", newFrontRightDeg);
            telemetry.addData("frontRightVoltz", hardwareMap.get(AnalogInput.class, "efrontRight").getVoltage());

            telemetry.addData("", "" + "\n");

            telemetry.addData("newFrontLeftRad", newFrontLeftRad);
            telemetry.addData("newFrontLeftDeg", newFrontLeftDeg);
            telemetry.addData("frontLeftVoltz", hardwareMap.get(AnalogInput.class, "efrontLeft").getVoltage());

            telemetry.addData("","" + "\n");

            telemetry.addData("newBackLeftRad", newBackLeftRad);
            telemetry.addData("newBackLeftDeg", newBackLeftDeg);
            telemetry.addData("backLeftVoltz", hardwareMap.get(AnalogInput.class, "ebackLeft").getVoltage());

            telemetry.addData("", "" + "\n");

            telemetry.addData("newBackRightRad", newBackRightRad);
            telemetry.addData("newBackRightDeg", newBackRightDeg);
            telemetry.addData("backRightVoltz", hardwareMap.get(AnalogInput.class, "ebackRight").getVoltage());

            telemetry.addData("", "" + "\n\n");


            telemetry.addData("frontRight", frontRightV + "rad");
            telemetry.addData("frontLeft", frontLeftV + "rad");
            telemetry.addData("backLeft", backLeftV + "rad");
            telemetry.addData("backRight", backRightV + "rad");
            telemetry.addData("pivot other", pivotEncoder.getCurrentPosition() + "rad");

            telemetry.addData("Pivot offset", pivot);

            telemetry.update();
        }
    }
}
