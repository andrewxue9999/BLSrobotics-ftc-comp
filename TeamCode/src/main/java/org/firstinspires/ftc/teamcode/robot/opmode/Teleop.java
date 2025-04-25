package org.firstinspires.ftc.teamcode.robot.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.Extendo;
import org.firstinspires.ftc.teamcode.robot.hardware.Pivot;
import org.firstinspires.ftc.teamcode.robot.hardware.Wrist;
import org.firstinspires.ftc.teamcode.robot.hardware.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.hardware.Claw;

@Config
@TeleOp(name="(use this fr) Teleop", group="Linear OpMode")
public class Teleop extends LinearOpMode {

    public enum ROBOT_STATE {
        INIT, BUCKET, CHAMBER, PICKUP
    }

    public static ROBOT_STATE robotState;
    public void setRobotState(ROBOT_STATE rs) {
        this.robotState = rs;
    }
    public ROBOT_STATE getRobotState() {
        return this.robotState;
    }


    private ElapsedTime runtime = new ElapsedTime();
    public final double TRACKWIDTH = 12.6378;
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    public static double fw_r = 4;
    public static double str_r = 4;

    SwerveDrivetrain drivetrain;
    Pivot pivot = new Pivot();
    Extendo extendo = new Extendo();
    Claw claw = new Claw();



    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        drivetrain = new SwerveDrivetrain();
//        drivetrain.init(hardwareMap);
//
//        fw = new SlewRateLimiter(fw_r);
//        str = new SlewRateLimiter(str_r);

        pivot.initialize(hardwareMap);

        while(!opModeIsActive()) {
            pivot.setRawPower(0.05);
        }

        extendo.initialize(hardwareMap);

        claw.initialize(hardwareMap);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

//            if (gamepad1.start) {
//                drivetrain.resetIMU();
//            }
//            // left stick to go forward, and right stick to turn.
//            double driveY = gamepad1.left_stick_y;
//            double driveX = -gamepad1.left_stick_x;
//
//            double azimuth = -gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now
//
//            drivetrain.read();
//
//            Pose drive = new Pose(driveY, driveX, azimuth);
//
//            drivetrain.set(drive);
//            drivetrain.write();
//            drivetrain.getTelemetry();

            pivot.update(telemetry);

            if (gamepad1.dpad_up) {
                pivot.setPivotState(Pivot.PIVOT_STATES.SCORING);
            } else if (gamepad1.dpad_down) {
                pivot.setPivotState(Pivot.PIVOT_STATES.PICKUP);
            }

            extendo.update(telemetry);

            if (gamepad1.a) {
                switch (claw.getClawState()) {
                    case CLOSED:
                        claw.setClawState(Claw.CLAW_STATES.OPEN);
                        break;
                    case OPEN:
                        claw.setClawState(Claw.CLAW_STATES.CLOSED);
                        break;
                }
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Gamepads", (driveX + "") + (driveY + ""));
//            telemetry.addData("Drivetrains", drivetrain.getTelemetry());

            telemetry.update();
        }
    }


    private double angleToServoPosition(double angle) {
        // angle 0-360 to servo position 0-1
        return Range.clip(angle / 360.0, 0.0, 1.0);
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }

}

