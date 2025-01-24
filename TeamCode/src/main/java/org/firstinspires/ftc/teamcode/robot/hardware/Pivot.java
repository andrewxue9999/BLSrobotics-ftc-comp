// gear ratios
// calculations for limits to not exceed 42 inch horizontal limit

package org.firstinspires.ftc.teamcode.robot.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

import java.util.Locale;

public class Pivot {
    private AnalogInput pivotReading;
    private AbsoluteAnalogEncoder pivotEncoder;
    private DcMotor pivot;
    private final double GEAR_RATIO = 300;
    private final double TPR = 28 * GEAR_RATIO; //ticks per rotation
    private final int topPosTicks = 2333; // temporary zero: extended all the way up (diagonal)
    private final double bottomPosTicks = -2758;
    private PIDFController PIDFcontroller;
    private final double P = 0.2;
    private final double I = 0.0;
    private final double D = 0.0;
    private final double F = 0.2;
    private static final double ENCODER_OFFSET = 1.2;
    private double currentPos;
    private double target;
    private final double TICKS_IN_DEGREES = 700.0 / 180;

    public void init(@NonNull HardwareMap hardwareMap, String pivotName, String analogInputName) {
       pivot = hardwareMap.get(DcMotor.class, pivotName); // should be "pivot" or "arm_motor0" ?

       pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       pivot.setDirection(DcMotor.Direction.REVERSE);

       PIDFcontroller = new PIDFController(P, I, D, F);
       PIDFcontroller.setPIDF(P, I, D, F);

       pivotReading = hardwareMap.get(AnalogInput.class, analogInputName);
       pivotEncoder = new AbsoluteAnalogEncoder(pivotReading, 3.3);
       pivotEncoder.zero(ENCODER_OFFSET);

    }


    public void goTo(String position) {
        switch (position) {
            case "scoring":
                target = topPosTicks;
                break;
            case "pickup":
                target = bottomPosTicks;
                break;
            default:
                target = 0; // "home" position
        }

        currentPos = Math.toDegrees(pivotEncoder.getCurrentPosition()) * TICKS_IN_DEGREES;

        PIDFcontroller.setPIDF(P, I, D, F);

        int currentPos = pivot.getCurrentPosition();
        double power = PIDFcontroller.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * F;

        power += ff;
        pivot.setPower(power);
    }


    // testing/debugging purposes only
    public void driveUp() {
        pivot.setPower(0.1);
    }

    // testing/debugging purposes only
    public void driveDown() {
        pivot.setPower(-0.1);
    }


    public String getTelemetry() {
      return String.format(Locale.ENGLISH, "current pivot position in ticks %d, current pivot position in degrees %.2f, target: %.2f, power: %.2f", pivot.getCurrentPosition(), currentPos, target, pivot.getPower());
    }
}
