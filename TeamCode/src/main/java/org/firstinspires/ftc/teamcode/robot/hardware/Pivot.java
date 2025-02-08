// gear ratios
// calculations for limits to not exceed 42 inch horizontal limit

package org.firstinspires.ftc.teamcode.robot.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

import java.util.Locale;

@Config
public class Pivot {
    // Pivot
    private AnalogInput pivotReading;
    private AbsoluteAnalogEncoder pivotEncoder;
    private DcMotor pivot;
    private final double GEAR_RATIO = 3.61 * 3.61 * 5.23 * 1.6;
    private final double TPR = 28 * GEAR_RATIO; //ticks per rotation

    private static double superBasePos = -973;
    private static int startIngConfig = -526;
    private static double pivotBucketPosTicks = 0.7;
    private static double pivotBottomPosTicks = -847;
    private PIDController pivotPIDcontroller;
    private static double pivotP = 0.01;
    private static double pivotI = 0.0;
    private static double pivotD = 0;//0.0001;
    private static double pivotF = 0.4;
    private static final double ENCODER_OFFSET = 0.2323;
    private double pivotCurrentPos;
    private double pivotTarget;

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
    private DcMotor extendo;
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

    public void init(@NonNull HardwareMap hardwareMap, String pivotName, String analogInputName, String extendoName, Gamepad gamepad) {
        pivot = hardwareMap.get(DcMotor.class, pivotName); // should be "pivot" or "arm_motor0" ?
        extendo = hardwareMap.get(DcMotor.class, extendoName); // DEFINE THE NAME IN HARDWARE MAP

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

        pivotReading = hardwareMap.get(AnalogInput.class, analogInputName);
        pivotEncoder = new AbsoluteAnalogEncoder(pivotReading, 3.3).zero(ENCODER_OFFSET);

        gp2 = gamepad;
//        pivotCurrentPos = 0.0;
    }

//    public void leoeo () {
//        pivotCurrentPos = pivot.getCurrentPosition();
//    }
    public void goTo() {

//        pivotEncoder.zero(ENCODER_OFFSET);

        // gamepad2 or gamepad1?
        boolean up = gp2.dpad_up;
        boolean down = gp2.dpad_down;
        boolean right = gp2.dpad_right;
        boolean left = gp2.dpad_left;

        boolean intakeLong = gp2.right_bumper;
        boolean intakeShort = gp2.left_bumper;

        if (up || down || right || left) {
            activated = true;
        } else {
            activated = false;
        }

        if (up) {
            pivotPosition = "scoring";
            extendoPosition = "high";
        } else if (down) {
            pivotPosition = "intake";
            extendoPosition = "intake";
        } else if (right) {
            pivotPosition = "scoring";
            extendoPosition = "low";
        }

        switch (pivotPosition) {
            case "scoring":
                pivotTarget = pivotBucketPosTicks;
                break;
            case "intake":
                pivotTarget = pivotBottomPosTicks;
                break;
            case "low":
                pivotTarget = pivotBucketPosTicks;
                break;
            case "starting":
                pivotTarget = startIngConfig;
                break;
//            default:
//                pivotTarget = 0; // "home" position
        }

        switch (extendoPosition) {
            case "retract":
                extendoTarget = extendoRetractedPosTicks;
                break;
            case "high":
                extendoTarget = extendoHighBucketPosTicks;
                break;
            case "low":
                extendoTarget = extendoLowBucketPosTicks;
                break;
            case "highChamber":
                extendoTarget = extendoHighChamberPosTicks;
                break;
            case "lowChamber":
                extendoTarget = extendoLowChamberPosTicks;
                break;
            case "intake":
                extendoTarget = extendoIntakePosTicks;
                break;
        }

        if (activated) {
//            pivotPIDcontroller.setSetPoint(pivotTarget);

            //while (!pivotPIDcontroller.atSetPoint()) {
                pivotPIDcontroller.setPID(pivotP, pivotI, pivotD);
                pivotCurrentPos = pivotEncoder.getCurrentPosition();

//                double error = pivotTarget - pivotCurrentPos;
//                double pivotPid = pivotPIDcontroller.calculate(0, error);// pivotCurrentPos, pivotTarget);
                double pivotPid = pivotPIDcontroller.calculate(pivotCurrentPos, pivotTarget);
//                double pivotPid = pivotPIDcontroller.calculate(pivot.getCurrentPosition());
//                pivot.setPower(pivotPid);


                double pivotFeedF = Math.cos(Math.toRadians(pivotTarget / TICKS_IN_DEGREES)) * pivotF;
                pivotPower = Range.clip(pivotPid + pivotFeedF, -1, 1);
                pivot.setPower(pivotPower);

            //}
            //pivot.setPower(0);
        }
    }


    public int posToTick(double pos)
    {
        return (int) (pos * TPR);
    }

    public void manualExtend(int increment) {
        this.extendoTarget += increment;
    }


    // testing/debugging purposes only
    public String getTelemetry() {
      return String.format(Locale.ENGLISH, "current motor pivot position in ticks %d, current pivot position converted to ticks from absEnc %.2f, target: %.2f, pivot power: %.2f, pivot position: %s, pivot target: %s. current extendo position in ticks %d, in degrees %.2f, target: %.2f, extendo power %.2f, extendo position: %s, extendo target: %s",
              pivot.getCurrentPosition(),
              pivotEncoder.getCurrentPosition(),
              pivotTarget,
              pivot.getPower(),
              pivotPosition,
              pivotTarget,
              extendo.getCurrentPosition(),
              extendoCurrentPos,
              extendoTarget,
              extendo.getPower(),
              extendoPosition,
              extendoTarget
      );
    }
}
