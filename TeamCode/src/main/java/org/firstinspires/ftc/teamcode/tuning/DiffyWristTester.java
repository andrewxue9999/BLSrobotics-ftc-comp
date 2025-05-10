package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.hardware.DifferentialWrist;

@Config
@TeleOp
public class DiffyWristTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Servo lServo;
    Servo rServo;


//    DifferentialWrist diffyWrist;





    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        diffyWrist = new DifferentialWrist();
//        diffyWrist.initialize(hardwareMap);

        lServo.setPosition(0.5);
        rServo.setPosition(0.5);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            double lspos = lServo.getPosition();
            double rspos = rServo.getPosition();




            if (gamepad1.dpad_left) {
                lServo.setPosition(lspos-0.05);
            }
            if (gamepad1.dpad_right) {
                lServo.setPosition(lspos+0.05);
            }
            if (gamepad1.x){
                rServo.setPosition(rspos-0.05);
            }
            if (gamepad1.b) {
                rServo.setPosition(rspos+0.05);
            }



            telemetry.addData("lservo", lspos);
            telemetry.addData("rservo", rspos);

            telemetry.update();
        }




    }
}
