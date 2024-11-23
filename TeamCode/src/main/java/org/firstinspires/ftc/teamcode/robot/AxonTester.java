
package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@TeleOp(name="AxonTester")
public class AxonTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //servos
    private CRServo frontRight;
    private CRServo frontLeft;
    private CRServo backLeft;
    private CRServo backRight;


    //encoders
    private AnalogInput axon1;
    private AnalogInput axon2;
    private AnalogInput axon3;
    private AnalogInput axon4;

    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(CRServo.class, "sfrontRight");
        frontLeft = hardwareMap.get(CRServo.class, "sfrontLeft");
        backLeft = hardwareMap.get(CRServo.class, "sbackLeft");
        backRight = hardwareMap.get(CRServo.class, "sbackRight");

        axon1 = hardwareMap.analogInput.get("efrontRight");
        axon2 = hardwareMap.analogInput.get("efrontLeft");
        axon3 = hardwareMap.analogInput.get("ebackLeft");
        axon4 = hardwareMap.analogInput.get("ebackRight");


        frontRight.setPower(1);

        TimeUnit.MILLISECONDS.sleep(800);

        frontRight.setPower(0);

        frontLeft.setPower(0.5);

        TimeUnit.SECONDS.sleep(2);

        frontLeft.setPower(0);



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("svoltage", "%4.2f, %4.2f, %4.2f %4.2f", axon1.getVoltage(), axon2.getVoltage(), axon3.getVoltage(), axon4.getVoltage());
            telemetry.update();
        }
    }}
