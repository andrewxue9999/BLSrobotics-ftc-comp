package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;


@Config
public class SwerveDrivetrain {
    public SwerveModule leftFront, leftBack, rightFront, rightBack;
    public SwerveModule[] swerveModules;
    private DcMotorEx mrightFront;
    private DcMotorEx mleftFront;
    private DcMotorEx mleftBack;
    private DcMotorEx mrightBack;

    private CRServo srightFront;
    private CRServo sleftFront;
    private CRServo sleftBack;
    private CRServo srightBack;

    private AnalogInput erightFront;
    private AnalogInput eleftFront;
    private AnalogInput eleftBack;
    private AnalogInput erightBack;
    private static double analogRange = 3.3;

    public static final double E_RIGHT_FRONT_OFFSET = -Math.PI/2 + 5.0932; //2.0449; // RADianz
    public static final double E_LEFT_FRONT_OFFSET = -Math.PI/2 + 5.1198; //1.14424;
    public static final double E_LEFT_BACK_OFFSET = -Math.PI/2 + 4.8342; // 1.487;
    public static final double E_RIGHT_BACK_OFFSET = -Math.PI/2 + 5.3826; //3.9565;

    public final double TRACKWIDTH = 12.6378; //in
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH/2, WHEELBASE/2);
    double[] wheelSpeeds = new double[4];
    double[] wheelAngles = new double[4];
    double max = 1.1;
    public boolean maintainHeading = false;

    public IMU imu;
    private double heading;

    public void init(@NonNull HardwareMap hardwareMap) {

        mleftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        mleftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        mrightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        mrightBack = hardwareMap.get(DcMotorEx.class, "backRight");

        mleftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        mleftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        mleftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mleftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mrightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mrightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleftFront = hardwareMap.get(CRServo.class, "sfrontLeft");
        sleftBack = hardwareMap.get(CRServo.class, "sbackLeft");
        srightFront = hardwareMap.get(CRServo.class, "sfrontRight");
        srightBack = hardwareMap.get(CRServo.class, "sbackRight");

        eleftFront = hardwareMap.get(AnalogInput.class, "efrontLeft");
        eleftBack = hardwareMap.get(AnalogInput.class, "ebackLeft");
        erightFront = hardwareMap.get(AnalogInput.class, "efrontRight");
        erightBack = hardwareMap.get(AnalogInput.class, "ebackRight");

// swap leftFront and leftBack (control hub naming issue?? wire connecting issue??)
        leftFront = new SwerveModule(mleftBack, sleftBack, new AbsoluteAnalogEncoder(eleftBack, analogRange).zero(E_LEFT_BACK_OFFSET));
        leftBack = new SwerveModule(mleftFront, sleftFront, new AbsoluteAnalogEncoder(eleftFront, analogRange).zero(E_LEFT_FRONT_OFFSET)); // removed .setInverted(true)
        rightFront = new SwerveModule(mrightFront, srightFront, new AbsoluteAnalogEncoder(erightFront, analogRange).zero(E_RIGHT_FRONT_OFFSET));
        rightBack = new SwerveModule(mrightBack, srightBack, new AbsoluteAnalogEncoder(erightBack, analogRange).zero(E_RIGHT_BACK_OFFSET));

        swerveModules = new SwerveModule[]{leftFront, leftBack, rightFront, rightBack};

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void read() {
        for (SwerveModule module : swerveModules) module.read();
    }

//    @Override
    public void set(Pose pose) {
        double x = pose.x; // "ly" x is controlled by left_stick_x
        double y = pose.y; // "lx" y is controlled by left_stick_y
        double rotation = pose.heading;


        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        x = -x * Math.sin(heading) + y * Math.cos(heading);
//        y = x * Math.cos(heading) + y * Math.sin(heading);
//        x = x * Math.cos(heading) - y * Math.sin(heading);
//        y = x * Math.sin(heading) + y * Math.cos(heading);

//        double r = Math.hypot(x, y);
//        double currentAngle = Math.atan(y / x);
//
//        y = r * Math.sin(currentAngle - heading);
//        x = r * Math.cos(currentAngle - heading);
        double tempX = x * Math.cos(heading) - y * Math.sin(heading);
        double tempY = x * Math.sin(heading) + y * Math.cos(heading);
//        x = Math.signum(x) * Math.abs(tempX);
//        y = Math.signum(y) * Math.abs(tempY);
        x = tempX;
        y = tempY;

        double a = x - rotation * (WHEELBASE / R),
                b = x + rotation * (WHEELBASE / R),
                c = y - rotation * (TRACKWIDTH / R),
                d = y + rotation * (TRACKWIDTH / R);

        wheelSpeeds = new double[]{Math.hypot(b, c), Math.hypot(b, d), Math.hypot(a, d), Math.hypot(a, c)};

        if (!maintainHeading) {
            wheelAngles = new double[]{Math.atan2(b, c), Math.atan2(b, d), Math.atan2(a, d), Math.atan2(a, c)}; // should be all in rads
        }

        max = wheelSpeeds[0];
        for (double i : wheelSpeeds) { // get max of wheelSpeeds
            if (i > max) max = i;
        }
    }


    public void write() {

        for (int i = 0; i < 4; i++) {
            SwerveModule m = swerveModules[i];

            if (Math.abs(max) > 1) wheelSpeeds[i] /= max; // scale everything to <=1 while maintaining proportions
            m.setMotorPower(Math.abs(wheelSpeeds[i]) + 0.1 * Math.signum(wheelSpeeds[i]));
            m.setTargetRotation((wheelAngles[i]) % (2*Math.PI));
            
            m.update();
        }
    }

    public String getTelemetry() {
        return leftFront.getTelemetry("leftFrontModule") + "\n" +
                leftBack.getTelemetry("leftRearModule") + "\n" +
                rightFront.getTelemetry("rightFrontModule") + "\n" +
                rightBack.getTelemetry("rightRearModule") + "\n" +
                String.format(Locale.ENGLISH, "IMU Yaw (Heading) = %.2f", heading);
    }

    public void resetIMU() {
        imu.resetYaw();
    }
}