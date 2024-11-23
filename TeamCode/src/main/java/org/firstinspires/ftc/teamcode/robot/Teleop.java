package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Pose;


@TeleOp(name="2024-25 Swerve TeleOp Code", group="Linear OpMode")
public class Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
//    public SwerveModule rightFront;
//    public SwerveModule leftFront;
//    public SwerveModule leftBack;
//    public SwerveModule rightBack;
    public SwerveModule[] swerveModules;

    public static final double E_FRONT_RIGHT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_FRONT_LEFT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_BACK_LEFT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_BACK_RIGHT_OFFSET = 1.1;//myArbitraryRadValue

    public final double TRACKWIDTH = 12.6378;
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
    private double voltage = 1;
    private PIDFController scontroller = new PIDFController(1.0, 0, 1.0, 0);
    public SwerveDrivetrain drivetrain;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        drivetrain = new SwerveDrivetrain(this);

        drivetrain.init();

//        //motors stuff
//        mrightFront  = hardwareMap.get(DcMotorEx.class, "frontRight");
//        mleftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        mleftBack  = hardwareMap.get(DcMotorEx.class, "backLeft");
//        mrightBack = hardwareMap.get(DcMotorEx.class, "backRight");
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        mrightFront.setDirection(DcMotor.Direction.FORWARD);
//        mleftFront.setDirection(DcMotor.Direction.REVERSE);
//        mleftBack.setDirection(DcMotor.Direction.REVERSE);
//        mrightBack.setDirection(DcMotor.Direction.FORWARD);
//
//        //servos stuff
//        srightFront = hardwareMap.get(CRServo.class, "sfrontRight");
//        sleftFront = hardwareMap.get(CRServo.class, "sfrontLeft");
//        sleftBack = hardwareMap.get(CRServo.class, "sbackLeft");
//        srightBack = hardwareMap.get(CRServo.class, "sbackRight");
//
////        PIDFController.PIDCoefficients scoefRightFront = new PIDFController.PIDCoefficients(1.0, 1.0,1.0);
//
//        //encoders stuff
//        erightFront =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontRight"), 3.3);
//        eleftFront =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontLeft"), 3.3);
//        eleftBack =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackLeft"), 3.3);
//        erightBack=  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackRight"), 3.3);
//
//        erightFront.zero(E_FRONT_RIGHT_OFFSET);
//        eleftFront.zero(E_FRONT_LEFT_OFFSET);
//        eleftBack.zero(E_BACK_LEFT_OFFSET);
//        erightBack.zero(E_BACK_RIGHT_OFFSET);
//
//
//        //actual module stuff
//        rightFront = new SwerveModule(mrightFront, srightFront, erightFront);
//        leftFront = new SwerveModule(mleftFront, sleftFront, eleftFront);
//        leftBack = new SwerveModule(mleftBack, sleftBack, eleftBack);
//        rightBack = new SwerveModule(mrightBack, srightBack, erightBack);
        //swerveModules = new SwerveModule[]{rightFront, leftFront, leftBack, rightBack};

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double rightFrontPower;
            double leftFrontPower;
            double leftBackPower;
            double rightBackPower;



            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;

            double azimuth = gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now


//            rightFrontPower = Range.clip(driveY - azimuth, -1.0, 1.0);
//            leftFrontPower = Range.clip(driveY + azimuth, -1.0, 1.0);
//            leftBackPower = Range.clip(driveY + azimuth, -1.0, 1.0);
//            rightBackPower = Range.clip(driveY - azimuth, -1.0, 1.0);

//            mrightFront.setPower(rightFrontPower);
//            mleftFront.setPower(leftFrontPower);
//            mleftBack.setPower(leftBackPower);
//            mrightBack.setPower(rightBackPower

            drivetrain.set(new Pose(driveY, driveX, azimuth));
            drivetrain.write();
            drivetrain.getTelemetry();
//            double testServoPower = gamepad1.left_stick_y;
////            double testServoPower = Range.clip(testServoEnableDouble, -1.0, 1.0); // don't question the naming convention


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Poopy", "Poop");
            telemetry.addData("Gamepads", (driveX + "") + (driveY + ""));
                telemetry.addData("Drivetrainz", drivetrain.getTelemetry());
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

