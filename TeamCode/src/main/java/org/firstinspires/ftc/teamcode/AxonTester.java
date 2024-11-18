
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="AxonTester")
public class AxonTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AnalogInput axon1 = null;
    private AnalogInput axon2;
    private AnalogInput axon3;
    private AnalogInput axon4;

    @Override
    public void runOpMode() {

        axon1 = hardwareMap.analogInput.get("efrontRight");
        axon2 = hardwareMap.analogInput.get("efrontLeft");
        axon3 = hardwareMap.analogInput.get("ebackLeft");
        axon4 = hardwareMap.analogInput.get("ebackRight");




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
