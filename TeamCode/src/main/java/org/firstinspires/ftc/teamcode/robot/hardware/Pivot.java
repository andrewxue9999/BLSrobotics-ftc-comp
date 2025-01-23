// pivot for the arm
// motor that makes it go in between certain angles; limits
// gear ratios
// calculations for limits to not exceed 42 inch horizontal limit
// account for how far and back it can go; limits
// do height (pythagorean theorem to calculate)
// set points (limits)

package org.firstinspires.ftc.teamcode.robot.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

public class Pivot {
    private DcMotor pivot;
    private final double GEAR_RATIO = 300;
    private final double TPR = 28 * GEAR_RATIO; //ticks per rotation

    public boolean scoring;
    public boolean pickup;
    public boolean home;
    private final int topPosTicks = 2333; // temporary zero: extended all the way up (diagonal)
    private final double bottomPosTicks = -2758;
    private PIDFController PIDFcontroller;
    private final double P = 0.2;
    private final double I = 0.0;
    private final double D = 0.0;
    private final double F = 0.2;
    private double error;

    public void init(@NonNull HardwareMap hardwareMap, String name) {
       pivot = hardwareMap.get(DcMotor.class, name);

       pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       pivot.setDirection(DcMotor.Direction.REVERSE);

       PIDFcontroller = new PIDFController(P, I, D, F);
       PIDFcontroller.setPIDF(P, I, D, F);
    }


    public void up(double power) { // for debugging purposes
        pivot.setTargetPosition(topPosTicks);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(power);
    }

    public void down(double power) {
        pivot.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(power);
    }

    public void driveUp() {
        pivot.setPower(0.1);
    }

    public void driveDown() {
        pivot.setPower(-0.1);
    }


    public String getTelemetry() {
      return String.format(Locale.ENGLISH, "current pivot position in ticks %d, error %.2f, power: %.2f, target ticks: %d", pivot.getCurrentPosition(), error, pivot.getPower(), pivot.getTargetPosition());
    }
}
