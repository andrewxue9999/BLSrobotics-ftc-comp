package org.firstinspires.ftc.teamcode.robot.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.Pivot;
import org.firstinspires.ftc.teamcode.robot.hardware.Wrist;
import org.firstinspires.ftc.teamcode.robot.hardware.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.hardware.Claw;

import java.util.Locale;

@Config
@TeleOp(name="(use this fr) Teleop", group="Linear OpMode")
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
//    public Pivot pivot;
    private Claw claw;

    private DcMotorEx pivot;
    private AnalogInput pivotAnalog;
    private AbsoluteAnalogEncoder pivotEnc;

    private PIDController pivotPid;

    /* limits 5.83 to 1.62 */
    public static double pivotTarget = 0.0;

//    public static double dP = 0.0;
//    public static double dI = 0.0;
//    public static double dD = 0.0;
    public static double F = 0.2;
    private final double GEAR_RATIO = 3.61 * 3.61 * 5.23 * 1.6;
    private final double TPR = 28 * GEAR_RATIO;


    // Pivot
    private AnalogInput pivotReading;
    private static double superBasePos = -973;
    private static int startIngConfig = -526;
    private static double pivotBucketPosTicks = 0.7;
    private static double pivotBottomPosTicks = -847;
    private PIDController pivotPIDcontroller;
    public static double pivotP = 1.7; // make this private later
    public static double pivotI = 0.0;
    public static double pivotD = 0.06;//0.0001;
    private static double pivotF = 0.4; // don't use this one; use "F"
    private static final double ENCODER_OFFSET = 0.1695;
    private double pivotCurrentPos;

    private double pivotPower;
    private final double TICKS_IN_DEGREES = TPR / 360;

    // Telescoping extendo thingy

    private final int extendoRetractedPosTicks = 0; // temporary zero: extended all the way up (diagonal)
    private final int extendoHighBucketPosTicks = -188;
    private final int extendoLowBucketPosTicks = -150;
    private final int extendoHighChamberPosTicks = 0;
    private final int extendoLowChamberPosTicks = 0;
    private final int extendoIntakePosTicks = 0;
    private final int extendoPickupPosTicks = 0;
    private DcMotorEx extendo;
    private PIDFController extendoPIDFcontroller;
    private final double extendoP = 0.0;
    private final double extendoI = 0.0;
    private final double extendoD = 0.0;
    private final double extendoF = 0.0;
    private double extendoCurrentPos;
    private double extendoTarget;
    private Gamepad gp2;
    private String pivotPosition = "";
    private String extendoPosition = "";
    boolean activated = false;
//    private Wrist wrist;

    private double pidPower;
    private double power; // for the pivot

    private Servo leftWrist;
    private Servo rightWrist;
    private Servo centralServo;





    /*$$$$$$$$$$$$$$$$$$$$$$$*/
    public static int PBUCKET = 890;
    public static int EHIGH = 440;


    public static int ELOW = 130;


    public static int PPickup = 24;
    public static int EPickup = 201;


    public static int PSTARTING = 308;
    public static int ESTARTING = 0;






    @Override
    public void runOpMode() {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drivetrain = new SwerveDrivetrain();
        drivetrain.init(hardwareMap);

        fw = new SlewRateLimiter(fw_r);
        str = new SlewRateLimiter(str_r);

//        pivot = new Pivot();
//        pivot.init(hardwareMap, "pivot", "poop", "extendo", gamepad2);

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        pivotAnalog = hardwareMap.get(AnalogInput.class, "poop");

        pivotEnc = new AbsoluteAnalogEncoder(pivotAnalog, 3.3);

        pivotPid = new PIDController(pivotP, pivotI, pivotD);
        extendo = hardwareMap.get(DcMotorEx.class, "extendo"); // DEFINE THE NAME IN HARDWARE MAP

        // Prolly should change this, I' just going off of what i think would work - victor
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotPIDcontroller = new PIDController(pivotP, pivotI, pivotD);
        pivotPIDcontroller.setPID(pivotP, pivotI, pivotD);

        extendoPIDFcontroller = new PIDFController(extendoP, extendoI, extendoD, extendoF);
        extendoPIDFcontroller.setPIDF(extendoP, extendoI, extendoD, extendoF);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        pivot.setDirection(DcMotor.Direction.REVERSE);


        leftWrist = hardwareMap.get(Servo.class, "lServo");
        rightWrist = hardwareMap.get(Servo.class, "rServo");
        centralServo = hardwareMap.get(Servo.class, "centerServo");

        leftWrist.scaleRange(0.21, 0.9);
//        rightWrist.scaleRange(0.44, 0.9);

        claw = new Claw();
        claw.init(hardwareMap, "claw");


        pivotEnc = pivotEnc.zero(ENCODER_OFFSET);

        // for auto decoding later: this is where "copying" ends

//        pivot.setTargetPosition(170);
//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pivot.setPower(0.5);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        pivot.setPower(0.6);
//        pivot.setTargetPosition(PSTARTING);
//        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

            doPivotPID();




//            if (gamepad2.dpad_up) {
//              pivotPid.setPID(pivotP, pivotI, pivotD);
//
//                pivotTarget = 1.76;
//
//              pidPower = pivotPid.calculate(pivotEnc.getCurrentPosition(), pivotTarget);
//              double ff = Math.cos(pivotEnc.getCurrentPosition()) * F;
//
//              power = pidPower + ff;
//              pivot.setPower(power);
//
//            }
//            else if (gamepad2.dpad_down) {
//
//                pivotTarget = 6.18;
//
//                pidPower = pivotPid.calculate(pivotEnc.getCurrentPosition() * TPR, pivotTarget * TPR);
//                double ff = Math.cos(pivotEnc.getCurrentPosition()) * F;
//                power = Range.clip(pidPower + ff, -1, 1);
//                pivot.setPower(power);
//
//                extendo.setTargetPosition(198);
//                extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                extendo.setPower(0.5);
//
//                leftWrist.setPosition(0.0); // 0.44
////                rightWrist.setPosition(0.0); // 0.44
//                centralServo.setPosition(0.0);
//            }
//            else if (gamepad2.dpad_right) {
//                pivotTarget = 1.76;
//
//                pidPower = pivotPid.calculate(pivotEnc.getCurrentPosition() * TPR, pivotTarget * TPR);
//                double ff = Math.cos(pivotEnc.getCurrentPosition()) * F;
//                power = Range.clip(pidPower + ff, -1, 1);
//                pivot.setPower(power);
//
//                extendo.setTargetPosition(94);
//                extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                extendo.setPower(0.5);
//
//                leftWrist.setPosition(1.0);
////                rightWrist.setPosition(0.9);
//                centralServo.setPosition(0.5);
//            }

//            pivot.goTo();

            if (gamepad2.a) {
                claw.toggle(runtime);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gamepads", (driveX + "") + (driveY + ""));
            telemetry.addData("Drivetrains", drivetrain.getTelemetry());

            telemetry.addData("pivot ticks", pivot.getCurrentPosition());
            telemetry.addData("extendo ticks", extendo.getCurrentPosition());

//            telemetry.addData("pos", pivotEnc.getCurrentPosition());
//            telemetry.addData("target", pivotTarget);
//            telemetry.addData("PidPower", pidPower);
//            telemetry.addData("power", power);
////            telemetry.addData("Pivot", pivot.getTelemetry());
//
//            telemetry.addData("Extendo position ticks", extendo.getCurrentPosition());
//            telemetry.addData("Extendo current", extendo.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Wrist position", String.format(Locale.ENGLISH, "left wrist position: %.2f, right wrist position: %.2f, central servo position: %.2f", leftWrist.getPosition(), rightWrist.getPosition(), centralServo.getPosition()));
            telemetry.update();
        }
    }

    private void doPivotPID() {
        if (gamepad2.dpad_up){
            pivot.setPower(.75);
            pivot.setTargetPosition(PBUCKET);

            extendo.setPower(0.75);
            extendo.setTargetPosition(EHIGH);

            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (gamepad2.dpad_left) {
            pivot.setPower(0.75);
            pivot.setTargetPosition(PBUCKET);

            extendo.setPower(0.75);
            extendo.setTargetPosition(ELOW);

            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (gamepad2.dpad_down) {
            pivot.setPower(0.75);
            pivot.setTargetPosition(PPickup);

            extendo.setPower(0.75);
            extendo.setTargetPosition(EPickup);

            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void climb() {

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

