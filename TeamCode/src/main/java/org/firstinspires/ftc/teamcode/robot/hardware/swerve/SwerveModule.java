package org.firstinspires.ftc.teamcode.robot.hardware.swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDController;
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
    public static double P = 0.2;//0.04; // can make this lower for less jerking/flickering/oscillations?
    public static double I = 0.0;
    public static double D = 0.0;
    public static double K_STATIC = 0.03;

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

    private double power = 0;
    private double error;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) { //, double r) { //, double sp, double si, double sd) {
        motor = m;
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo = s;
        encoder = e;

        scontroller = new PIDController(P, I, D);

        scontroller.setPID(P, I, D);
    }

    public void read() {
        position = encoder.getCurrentPosition();
    }


    public void update() {
        scontroller.setPID(P, I, D);
        double target = getTargetRotation();
        double currentPos = getModuleRotation();
        error = normalizeRadians(target - currentPos);

        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } // reverse direction ^
        else {
            wheelFlipped = false;
        }

        error = normalizeRadians(target - currentPos);

        double power = Range.clip(scontroller.calculate(0, error), -1, 1);
        if (Double.isNaN(power)) power = 0; // set 0 if null calculation
        if (Math.abs(error) <= DEADBAND) {
            power += 0;
        } else {
            power += K_STATIC * Math.signum(power);
        }
        servo.setPower(power);
        //         servo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
    }

    public void setInverted(boolean invert) {
        encoder.setInverted(invert);
    }

    private double getModuleRotation() {
        return normalizeRadians(position);
    }

    private double getTargetRotation() {
        return target;
    }

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
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
        return String.format(Locale.ENGLISH, "%s: Motor Flipped: %b \ncurrent position %.2f target position %.2f motor power = %.2f error=%.2f", name, wheelFlipped, getModuleRotation(), getTargetRotation(), power, error);
    }

}