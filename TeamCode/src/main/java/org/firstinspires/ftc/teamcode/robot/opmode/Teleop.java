package org.firstinspires.ftc.teamcode.robot.opmode;

import android.text.ParcelableSpan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftccommon.internal.RunOnBoot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.DifferentialWrist;
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
        INIT, BUCKET, CHAMBER, HUNTING, PICKUP, CHECKING_PICKUP, POST_PICKUP
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

    public Gamepad previousGamepad1 = new Gamepad();

    SwerveDrivetrain drivetrain;
    Pivot pivot = new Pivot();
    Extendo extendo = new Extendo();
    Claw claw = new Claw();
    DifferentialWrist diffy = new DifferentialWrist();

    ElapsedTime pivotTimer = new ElapsedTime();
    private static final double DOWN_TIME = 0.3;

    ElapsedTime clawTimer = new ElapsedTime();
    private static final double CLOSE_TIME = 0.5;

    ElapsedTime extendoTimer = new ElapsedTime();
    private static final double RETRACT_TIME = 0.4;





    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drivetrain = new SwerveDrivetrain();
        drivetrain.init(hardwareMap);

        fw = new SlewRateLimiter(fw_r);
        str = new SlewRateLimiter(str_r);

        pivot.initialize(hardwareMap);
        pivotTimer.reset();

        extendo.initialize(hardwareMap);
        extendoTimer.reset();

        claw.initialize(hardwareMap);
        clawTimer.reset();

        diffy.initialize(hardwareMap);

        while (opModeInInit()) {
            pivot.setPivotState(Pivot.PIVOT_STATES.INIT);
            pivot.update(telemetry);
            telemetry.update();
        }


        waitForStart();
        runtime.reset();

        setRobotState(ROBOT_STATE.POST_PICKUP);

        while (opModeIsActive()) {

            // left stick to go forward, and right stick to turn.
            double driveY = gamepad1.left_stick_y;
            double driveX = -gamepad1.left_stick_x;

            double azimuth = -gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now

            drivetrain.read();

            Pose drive = new Pose(driveY, driveX, azimuth);

            drivetrain.set(drive);
            drivetrain.write();
            drivetrain.getTelemetry();

            if (gamepad1.dpad_up) {
                setRobotState(ROBOT_STATE.BUCKET);
            } else if (gamepad1.dpad_down) {
                setRobotState(ROBOT_STATE.HUNTING);
                extendo.setExtendoState(Extendo.EXTENDO_STATES.PICKUP);

                extendoTimer.reset();
            }

            switch(getRobotState()) {
                case BUCKET:
                    pivot.setPivotState(Pivot.PIVOT_STATES.SCORING);
                    extendo.setExtendoState(Extendo.EXTENDO_STATES.BHIGH);
                    diffy.setWristState(DifferentialWrist.WRIST_STATE.BUCKET);

                    if (gamepad1.b) {
                        claw.setClawState(Claw.CLAW_STATES.OPEN);
                    }
                    break;
                case HUNTING:
                    claw.setClawState(Claw.CLAW_STATES.OPEN);
                    diffy.setWristState(DifferentialWrist.WRIST_STATE.PICKUP_PARA);

                    if (extendoTimer.seconds() >= RETRACT_TIME) {
                        pivot.setPivotState(Pivot.PIVOT_STATES.HUNTING);
                        if (gamepad1.left_bumper) {
                            switch (diffy.getWristState()) {
                                case PICKUP_PARA:
                                    diffy.setWristState(DifferentialWrist.WRIST_STATE.PIKCUP_VERT);
                                    break;
                                case PIKCUP_VERT:
                                    diffy.setWristState(DifferentialWrist.WRIST_STATE.PICKUP_PARA);
                                    break;
                            }
                        }

                        if (gamepad1.a) {
                            setRobotState(ROBOT_STATE.PICKUP);

                            pivot.setPivotState(Pivot.PIVOT_STATES.PICKUP);

                            pivotTimer.reset();
                        }

                    }

                    break;
                case PICKUP:

                    if(pivotTimer.seconds() >= DOWN_TIME){
                        claw.setClawState(Claw.CLAW_STATES.CLOSED);

                        setRobotState(ROBOT_STATE.CHECKING_PICKUP);

                        clawTimer.reset();
                    }

                    break;
                case CHECKING_PICKUP:

                    if (clawTimer.seconds() >= CLOSE_TIME) {
                        setRobotState(ROBOT_STATE.POST_PICKUP);
                    }
                    break;
                case POST_PICKUP:
                    pivot.setPivotState(Pivot.PIVOT_STATES.POST_PICKUP);
                    extendo.setExtendoState(Extendo.EXTENDO_STATES.RETRACTED);
                    break;
            }

            pivot.update(telemetry);
            claw.update(telemetry);
            extendo.update(telemetry);
            diffy.update(telemetry);
            


            // Show the elapsed game time and wheel power.
            telemetry.addData("dpad up down", gamepad1.dpad_up || gamepad1.dpad_down);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gamepads", (driveX + "") + (driveY + ""));
            telemetry.addData("Drivetrains", drivetrain.getTelemetry());

            telemetry.update();

            previousGamepad1 = this.gamepad1;
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

