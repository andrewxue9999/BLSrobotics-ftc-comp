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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

public class Pivot {
    private DcMotor pivot;
  private final double DRIVE_RATIO = (52 * 2 * 2) / 18.0; //208/18
    private final double TPR = 28 * DRIVE_RATIO; //ticks per 1 wheel irl rotation;

    private boolean isAtTop = false;
    private final double topPosTicks = -586; // temporary zero: extended all the way up (diagonal)
    private final double bottomPosTicks = -2758;

    private final double topPos = TPR * Math.PI / 2;
    private final double bottomPos = TPR * Math.PI;
    private PIDFController PIDFcontroller;
    private final double P = 0.2;
    private final double I = 0.0;
    private final double D = 0.0;
    private final double F = 0.2;
    private double error;
    private double placeholderPower;

    public void init(@NonNull HardwareMap hardwareMap, String name) {
       pivot = hardwareMap.get(DcMotor.class, name);

       pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       PIDFcontroller = new PIDFController(P, I, D, F);
       PIDFcontroller.setPIDF(P, I, D, F);
  }

    public void toggle(boolean goToTop) {

        // power = 0 (class variable; declared outside of the method)
        // if goToTop, power = PIDF calculated value
        // pivot.setPower(power)
        //

        if (goToTop) { // may need to implement a while loop for this soon; otherwise, it will only go up partially
            error = topPosTicks - pivot.getCurrentPosition();

            double power = Range.clip(PIDFcontroller.calculate(0, error), -1, 1);

            if (Double.isNaN(power)) {
                power = 0;
            }

            pivot.setPower(0.5);
            placeholderPower = power;

//            pivot.setPower(power);
            isAtTop = true;
        }
        else if (!goToTop) {
            error = bottomPosTicks - pivot.getCurrentPosition();

            double power = Range.clip(PIDFcontroller.calculate(0, error), -1, 1);
            if (Double.isNaN(power)) {
                power = 0;
            }
            pivot.setPower(0.5);
            placeholderPower = power;

//            pivot.setPower(power);
            isAtTop = false;
        }
    }

    public void setPower(double power) { // for debugging purposes
        pivot.setPower(power);
    }

    public boolean atTop() {
        return isAtTop;
    }

  public String getTelemetry() {
      return String.format(Locale.ENGLISH, "current pivot position in ticks %d, error %.2f, at top: %b, power: %.2f", pivot.getCurrentPosition(), error, isAtTop, pivot.getPower());
  }
}
