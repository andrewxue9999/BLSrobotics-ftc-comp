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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

import java.util.Locale;

@Config
public class Pivot {

    Telemetry telemetry;

    // Pivot
    private AnalogInput pivotReading;
    private AbsoluteAnalogEncoder pivotEncoder;
    private DcMotor pivot;
    private final double GEAR_RATIO = 3.61 * 3.61 * 5.23 * 1.6;
    private final double TPR = 28 * GEAR_RATIO; //ticks per rotation

    public static int superBaseTicks = -973;
    public static int startIngConfig = -526;
    public static int pivotTopPosTicks = 18; // temporary zero: extended all the way up (diagonal)
    public static int pivotBottomPosTicks = -847;
    private PIDController pivotPIDcontroller;
    public static double pivotP = 0.01;
    public static double pivotI = 0.0;
    public static double pivotD = 0.0001;
    public static double pivotF = 0.4;
    private static final double ENCODER_OFFSET = 0.3161;
    public double pivotCurrentPos;
    public double pivotTarget;

    public double pivotPower;
    private final double TICKS_IN_DEGREES = TPR / 360;

    // Telescoping extendo thingy

    private final int extendoRetractedPosTicks = 0; // temporary zero: extended all the way up (diagonal)
    private final int extendoHighBucketPosTicks = 0;
    private final int extendoLowBucketPosTicks = 0;
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

    public void init(@NonNull HardwareMap hardwareMap, String pivotName, String analogInputName, String extendoName) {
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

        pivotReading = hardwareMap.get(AnalogInput.class, analogInputName);
        pivotEncoder = new AbsoluteAnalogEncoder(pivotReading, 3.3);
        pivotEncoder.zero(ENCODER_OFFSET);

    }

//    public void leoeo () {
//        pivotCurrentPos = pivot.getCurrentPosition();
//    }
    public void goTo(String pivotPosition, String extendoPosition) {

        switch (pivotPosition) {
            case "scoring":
                pivotTarget = pivotTopPosTicks;
                break;
            case "pickup":
                pivotTarget = pivotBottomPosTicks;
                break;
            case "starting":
                pivotTarget = startIngConfig;
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
            case "pickup":
                extendoTarget = extendoPickupPosTicks;
        }

        pivotPIDcontroller.setPID(pivotP, pivotI, pivotD);

        while (pivot.getCurrentPosition() > (pivotTarget + 10) || pivot.getCurrentPosition() < (pivotTarget - 10)) {
            pivotCurrentPos = pivot.getCurrentPosition();
            double pivotPid = pivotPIDcontroller.calculate(pivotCurrentPos, pivotTarget);
            double pivotFeedF = Math.cos(Math.toRadians(pivotTarget / TICKS_IN_DEGREES)) * pivotF;
            pivotPower = pivotPid + pivotFeedF;
            pivot.setPower(pivotPower);
        }








    }

    public void manualExtend(int increment) {
        this.extendoTarget += increment;
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
      return String.format(Locale.ENGLISH, "current motor pivot position in ticks %d, current pivot position converted to ticks from absEnc %.2f, target: %.2f, power: %.2f. current extendo position in ticks %d, in degrees %.2f, target: %.2f, power %.2f",
              pivot.getCurrentPosition(),
              pivotCurrentPos,
              pivotTarget,
              pivotPower,
              extendo.getCurrentPosition(),
              extendoCurrentPos,
              extendoTarget,
              extendo.getPower()
      );
    }
}
