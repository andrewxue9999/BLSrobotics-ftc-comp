package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.robot.hardware.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.SquidController;

@Config
@TeleOp(name = "TeleOp Swerve", group = "Drive")
public class Swerce extends OpMode {
    private double oldTime = 0.0;

    private double voltz;

    private SwerveDrivetrain drivetrain;
    private GoBildaPinpointDriver odo;

    private double currentHeading;
    private double supposedHeading = 0.0;
    private SquidController hController;
    public static double hKSQ = 0.5;
    public static double hD = 0.0;
    public static double hI = 0.0;

    SlewRateLimiter fwdSlr = new SlewRateLimiter(4);
    SlewRateLimiter strSlr = new SlewRateLimiter(4);

    @Override
    public void init() {
        drivetrain = new SwerveDrivetrain(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        odo.setOffsets(0.0, 38.6    , DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        hController = new SquidController(hKSQ, hI, hD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version       Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();

        if (gamepad1.start) {
            odo.resetPosAndIMU();
        }

        double y = -gamepad1.left_stick_y; // Y is inverted
        double x = gamepad1.left_stick_x;
        double turn = joystickScalar(gamepad1.right_stick_x, 0.01);

        Vector2d trans = new Vector2d(x, y); //DIS DA VECTOR2D FROM TEAMCODE.UTIL

        currentHeading = Angle.normDelta(odo.getHeading(AngleUnit.RADIANS));
//        turn = Angle.normDelta(turn);

        double headingError = Angle.normDelta(turn - currentHeading);

        double headingCorrection = -hController.calculate(0, headingError);

        trans.rotated(currentHeading);

        Pose drive = new Pose(fwdSlr.calculate(trans.getX()), strSlr.calculate(trans.getY()), turn);

        drivetrain.maintainHeading = Math.abs(gamepad1.left_stick_x) < 0.05 &&
                Math.abs(gamepad1.left_stick_y) < 0.05 &&
                Math.abs(gamepad1.right_stick_x) < 0.05;

        drivetrain.read();
        drivetrain.setDrivePower(drive);
        drivetrain.update(telemetry);

        drivetrain.getTelemetry(telemetry);

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        telemetry.addData("frequency", frequency);

        telemetry.update();
    }


    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }
}
