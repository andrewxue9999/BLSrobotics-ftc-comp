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

        lServo = hardwareMap.get(Servo.class, "lServo");
        rServo = hardwareMap.get(Servo.class, "rServo");

        double lServoPos = 0.5; // Initial positions
        double rServoPos = 0.5;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;
        boolean prevX = false;
        boolean prevB = false;


        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            // --- Left Servo ---
            if (gamepad1.dpad_left && !prevDpadLeft) {
                lServoPos -= 0.05;
            }
            if (gamepad1.dpad_right && !prevDpadRight) {
                lServoPos += 0.05;
            }
            lServoPos = Math.max(0, Math.min(1, lServoPos)); // Clamp between 0 and 1
            lServo.setPosition(lServoPos);

            // --- Right Servo ---
            if (gamepad1.x && !prevX) {
                rServoPos -= 0.05;
            }
            if (gamepad1.b && !prevB) {
                rServoPos += 0.05;
            }
            rServoPos = Math.max(0, Math.min(1, rServoPos));
            rServo.setPosition(rServoPos);

            // --- Store previous button states ---
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            prevX = gamepad1.x;
            prevB = gamepad1.b;

            telemetry.addData("lservo", lServoPos);
            telemetry.addData("rservo", rServoPos);
            telemetry.update();
        }




    }
}
