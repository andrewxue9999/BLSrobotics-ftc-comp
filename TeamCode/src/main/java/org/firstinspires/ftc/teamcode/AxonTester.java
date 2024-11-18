
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="AxonTester")
public class AxonTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AnalogInput axon = null;

    @Override
    public void runOpMode() {

        axon = hardwareMap.analogInput.get("efrontRight");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("front right voltage", "%4.2f", axon.getVoltage());
            telemetry.update();
        }
    }}
