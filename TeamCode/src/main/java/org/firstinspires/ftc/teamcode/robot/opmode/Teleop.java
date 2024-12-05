package org.firstinspires.ftc.teamcode.robot.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

@TeleOp(name="2024-25 Swerve TeleOp Code", group="Linear OpMode")
public class Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public final double TRACKWIDTH = 12.6378;
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    public static double fw_r = 8;
    public static double str_r = 8;
    public SwerveDrivetrain drivetrain;
    public boolean isRight;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drivetrain = new SwerveDrivetrain();
        drivetrain.init(hardwareMap);

        fw = new SlewRateLimiter(fw_r);
        str = new SlewRateLimiter(str_r);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // left stick to go forward, and right stick to turn.
            double driveY = gamepad1.left_stick_y;
            double driveX = -gamepad1.left_stick_x;

            double azimuth = gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now

            drivetrain.read();

            drivetrain.maintainHeading = Math.abs(driveY) < 0.002 && Math.abs(driveX) < 0.002 && Math.abs(azimuth) < 0.002;

            Pose drive = new Pose(new Point(joystickScalar(driveY, 0.001), joystickScalar(driveX, 0.001)), joystickScalar(azimuth, 0.01));
            drive = new Pose(fw.calculate(drive.x), str.calculate(drive.y), drive.heading); // yes, these two lines can be simplified to one, but keep it this way for now.

            if (drive.heading > 0) {
                isRight = true;
            }
            else if (drive.heading < 0) {
                isRight = false;
            }
//            drivetrain.set(new Pose(driveY, driveX, azimuth));
            drivetrain.set(drive, isRight);
            drivetrain.write();
            drivetrain.getTelemetry();
//            double testServoPower = gamepad1.left_stick_y;
////            double testServoPower = Range.clip(testServoEnableDouble, -1.0, 1.0); // don't question the naming convention


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gamepads", (driveX + "") + (driveY + ""));
                telemetry.addData("Drivetrains", drivetrain.getTelemetry());
//            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back", leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
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

