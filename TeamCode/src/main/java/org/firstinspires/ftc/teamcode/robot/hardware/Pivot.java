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

public class Pivot {
    private DcMotor pivot;
  private final double DRIVE_RATIO = (52 * 2 * 2) / 18.0; //208/18
    private final double TPR = 28 * DRIVE_RATIO; //ticks per 1 wheel irl rotation;

    private boolean isAtTop = false;

    private final double topPos = TPR * Math.PI / 2;
    private final double bottomPos = TPR * Math.PI;
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

       PIDFcontroller = new PIDFController(P, I, D, F);
       PIDFcontroller.setPIDF(P, I, D, F);
  }

    public void toggle(boolean goToTop) {
        if (goToTop) {
            error = normalizeRadians(topPos - pivot.getCurrentPosition());

            double power = Range.clip(PIDFcontroller.calculate(0, error), -1, 1);

            if (Double.isNaN(power)) {
                power = 0;
            }

            pivot.setPower(power);
        }
        else if (!goToTop) {
            error = normalizeRadians(topPos - pivot.getCurrentPosition());

            double power = Range.clip(PIDFcontroller.calculate(0, error), -1, 1);
            if (Double.isNaN(power)) {
                power = 0;
            }
            pivot.setPower(power);
        }
    }

    public boolean atTop() {
        return isAtTop;
    }

  public String getTelemetry(String name) {
      return String.format("Open %S", name);
  }
}
