package org.firstinspires.ftc.teamcode.robot.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.hardware.Pivot;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.hardware.Claw;
@TeleOp(name="skibidiTeleop", group="Linear OpMode")
public class Teleop extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    public final double TRACKWIDTH = 12.6378;
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    public static double fw_r = 4;
    public static double str_r = 4;
    public SwerveDrivetrain drivetrain;
    public Pivot pivot;
    private Claw claw;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drivetrain = new SwerveDrivetrain();
        drivetrain.init(hardwareMap);

        fw = new SlewRateLimiter(fw_r);
        str = new SlewRateLimiter(str_r);

        pivot = new Pivot();
        pivot.init(hardwareMap, "pivot", "poop");

        claw = new Claw();
        claw.init(hardwareMap, "claw");
        
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.start) {
                drivetrain.resetIMU();
            }
            // left stick to go forward, and right stick to turn.
            double driveY = gamepad1.left_stick_y;
            double driveX = -gamepad1.left_stick_x;

            double azimuth = -gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now

            drivetrain.read();

            drivetrain.maintainHeading = Math.abs(driveY) < 0.002 && Math.abs(driveX) < 0.002 && Math.abs(azimuth) < 0.002;

            Pose drive = new Pose(new Point(joystickScalar(driveY, 0.001), joystickScalar(driveX, 0.001)), joystickScalar(azimuth, 0.01));
            drive = new Pose(fw.calculate(drive.x), str.calculate(drive.y), drive.heading); // yes, these two lines can be simplified to one, but keep it this way for now.


            drivetrain.set(new Pose(driveX, driveY, azimuth));
            drivetrain.set(drive);
            drivetrain.write();
            drivetrain.getTelemetry();

            if (gamepad1.dpad_up) {
                pivot.goTo("scoring");
            } else if (gamepad1.dpad_down) {
                pivot.goTo("pickup");
            }

            if (gamepad1.a) {
                claw.toggle(runtime);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gamepads", (driveX + "") + (driveY + ""));
            telemetry.addData("Drivetrains", drivetrain.getTelemetry());
//            telemetry.addData("Pivot position", pivot.getTelemetry());
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

