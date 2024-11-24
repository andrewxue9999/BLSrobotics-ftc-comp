package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

import java.util.Locale;

@Config // config lets us access and edit through FTC Dashboard in real time without rebuilding every time
public class SwerveModule {
    // drive gears, steering gears, drive motor, azimuth motor, absolute encoder
    public static double P = 0.31;//0.04;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double K_STATIC = 0.0;

    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDController scontroller;
    private final double WHEEL_RAD = 2.67717; //inches might change irl due to wheel squish
    private final double DRIVE_RATIO = (52 * 2 * 2) / 18.0; //208/18
    private final double AZIMUTH_RATIO = 1.0; //for now
    private final double TPR = 28 * DRIVE_RATIO; //ticks per 1 wheel irl rotation;

    private final double OFFSET = .0; // Math.PI / 2.0;    // TODO CHANGE BACK TO ZERO WHEN WORKS. This offset makes it so zero position is at 90 degrees

    private final double DEADBAND = .03;

    public boolean wheelFlipped = false;
    private double position;
    private double target;

    public double lastMotorPower = 0; // IGNORE BUT DO NOT REMOVE

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) { //, double r) { //, double sp, double si, double sd) {
        motor = m;
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        servo = s;
        //((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000)); // change these numbers later??


        encoder = e;

        scontroller = new PIDController(P, I, D);
        scontroller.setPID(P, I, D);


    }

    public void read() {
        position = encoder.getCurrentPosition() ;
    }

    double error = 0;

    public void update() {

        double targetPos = getTargetRotation();
        double currentPos = getModuleRotation();
        error = (targetPos - currentPos);

//        if (Math.abs(error) > Math.PI / 2) {
//            target = normalizeRadians(targetPos - Math.PI);
//            error = normalizeRadians(target - currentPos);
//            wheelFlipped = true;
//        } // reverse direction

        double power = Range.clip(scontroller.calculate(currentPos, targetPos), -1, 1);
        if (Double.isNaN(power)) power = 0; // set 0 if null calculation
        if (Math.abs(error) <= DEADBAND) {
            power = 0;
        }
        servo.setPower(power);
        //         servo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));

    }

    private double getModuleRotation() {
        return (position); // used to have normalizeRadians()
    }

    private double getTargetRotation() {
        return target;
    }

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
//        lastMotorPower = power;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.target = (target); // used to have normalizeRadians()
    }

    public double getWheelPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }

    private double encoderTicksToInches(double ticks) {
        return WHEEL_RAD * 2 * Math.PI * DRIVE_RATIO * ticks / TPR;
    }

    public String getTelemetry(String name) {
        return String.format(Locale.ENGLISH, "%s: Motor Flipped: %b \ncurrent position %.2f target position %.2f motor power = %.2f error=%.2f", name, wheelFlipped, getModuleRotation(), getTargetRotation(), lastMotorPower, error);
    }
//
//    public void azimtuh(double pos) { //in RADianz
//        scontroller.setSetPoint(pos);
//
//        while (!scontroller.atSetPoint()) {
//            double output = scontroller.calculate(
//                    encoder.getCurrentPosition(), pos  // the measured value and the setpoint
//            );
//
//            servo.setPower(output);
//        }
//
//    }

//    public void drive (double pos, double x, double y) {
//
//    }

}