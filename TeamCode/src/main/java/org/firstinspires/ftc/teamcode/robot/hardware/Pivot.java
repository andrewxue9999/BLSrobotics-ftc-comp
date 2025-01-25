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

    // Pivot
    private AnalogInput pivotReading;
    private AbsoluteAnalogEncoder pivotEncoder;
    private DcMotor pivot;
    private final double GEAR_RATIO = 300;
    private final double TPR = 28 * GEAR_RATIO; //ticks per rotation
    private final int pivotTopPosTicks = 2333; // temporary zero: extended all the way up (diagonal)
    private final int pivotBottomPosTicks = -2758;
    private PIDFController pivotPIDFcontroller;
    private final double pivotP = 0.2;
    private final double pivotI = 0.0;
    private final double pivotD = 0.0;
    private final double pivotF = 0.2;
    private static final double ENCODER_OFFSET = 1.2;
    private double pivotCurrentPos;
    private double pivotTarget;
    private final double TICKS_IN_DEGREES = 700.0 / 180;

    // Telescoping extendo thingy

    private final int extendoRetractedPosTicks = 0; // temporary zero: extended all the way up (diagonal)
    private final int extendoHighBucketPosTicks = 0;
    private final int extendoLowBucketPosTicks = 0;
    private final int extendoHighChamberPosTicks = 0;
    private final int extendoLowChamberPosTicks = 0;
    private final int extendoIntakePosTicks = 0;
    private DcMotor extendo;
    private PIDFController extendoPIDFcontroller;
    private final double extendoP = 0.0;
    private final double extendoI = 0.0;
    private final double extendoD = 0.0;
    private final double extendoF = 0.0;
    private double extendoCurrentPos;
    private double extendoTarget;

    public void init(@NonNull HardwareMap hardwareMap, String pivotName, String analogInputName, String extendoName) {
        pivot = hardwareMap.get(DcMotor.class, pivotName); // should be "pivot" or "arm_motor0" ?
        extendo = hardwareMap.get(DcMotor.class, extendoName); // DEFINE THE NAME IN HARDWARE MAP

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setDirection(DcMotor.Direction.REVERSE);

        // Prolly should change this, I' just going off of what i think would work - victor
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotPIDFcontroller = new PIDFController(pivotP, pivotI, pivotD, pivotF);
        pivotPIDFcontroller.setPIDF(pivotP, pivotI, pivotD, pivotF);

        extendoPIDFcontroller = new PIDFController(extendoP, extendoI, extendoD, extendoF);
        extendoPIDFcontroller.setPIDF(extendoP, extendoI, extendoD, extendoF);

        pivotReading = hardwareMap.get(AnalogInput.class, analogInputName);
        pivotEncoder = new AbsoluteAnalogEncoder(pivotReading, 3.3);
        pivotEncoder.zero(ENCODER_OFFSET);

    }


    public void goTo(String pivotPosition, String extendoPosition) {
        switch (pivotPosition) {
            case "scoring":
                pivotTarget = pivotTopPosTicks;
                break;
            case "pickup":
                pivotTarget = pivotBottomPosTicks;
                break;
            default:
                pivotTarget = 0; // "home" position
        }

        switch (extendoPosition) {
            case "retract":
                extendoTarget = extendoRetractedPosTicks;
                break;
            case "highBucket":
                extendoTarget = extendoHighBucketPosTicks;
                break;
            case "lowBucket":
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

        pivotCurrentPos = Math.toDegrees(pivotEncoder.getCurrentPosition()) * TICKS_IN_DEGREES;

        pivotPIDFcontroller.setPIDF(pivotP, pivotI, pivotD, pivotF);
        extendoPIDFcontroller.setPIDF(extendoP, extendoI, extendoD, extendoF);

        pivotCurrentPos = pivot.getCurrentPosition(); // This is before the absolute encoder
        double pivotPower = pivotPIDFcontroller.calculate(pivotCurrentPos, pivotTarget);
        double pivotff = Math.cos(Math.toRadians(pivotTarget / TICKS_IN_DEGREES)) * pivotF;

        double extendoPower = extendoPIDFcontroller.calculate(extendoCurrentPos, extendoTarget);
        double extendoff = Math.cos(Math.toRadians(extendoTarget / TICKS_IN_DEGREES)) * extendoF;

        pivotPower += pivotff;
        pivot.setPower(pivotPower);

        extendoPower += extendoff;
        extendo.setPower(extendoPower);
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
      return String.format(Locale.ENGLISH, "current pivot position in ticks %d, current pivot position in degrees %.2f, target: %.2f, power: %.2f. current extendo position in ticks %d, in degrees %.2f, target: %.2f, power %.2f",
              pivot.getCurrentPosition(),
              pivotCurrentPos,
              pivotTarget,
              pivot.getPower(),
              extendo.getCurrentPosition(),
              extendoCurrentPos,
              extendoTarget,
              extendo.getPower()
      );
    }
}
