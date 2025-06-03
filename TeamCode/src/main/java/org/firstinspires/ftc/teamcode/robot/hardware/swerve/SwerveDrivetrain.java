package org.firstinspires.ftc.teamcode.robot.hardware.swerve;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import com.acmerobotics.roadrunner.util.Angle;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.SquidController;


import java.util.ArrayList;
import java.util.List;

@Config
public class SwerveDrivetrain {
    private boolean locked = false;
    public boolean maintainHeading = false;

    public double supposedHeading = 0;

    private final SwerveModule[] modules;
    private final SwerveModule.SwerveModuleState[] states;

    private double speeds[] = new double[4];
    private double angles [] = new double[4];

    Pose2D pos;
    /*WE ARE USING THE ROADRUNNER KOTLIN POSE ONLY FROM NOW ON BUT SINCE
    .GETPOSITION() RETURNS A ROBOTCORE POSE 2D WE HAVE TO USE THAT ONE HERE
     */

    private final double TW = 12.637;
    private  final double WB = 12.637   ;
    private final double R = hypot(TW/2, WB/2);

    private final double[] MIN_WHEEL_POWER = {0.06, 0.06, 0.06, 0.06};

    private SwerveModule frontLeft, frontRight, backLeft, backRight;

    private DcMotorEx mfrontLeft, mfrontRight, mbackLeft, mbackRight;

    private CRServo sfrontLeft, sfrontRight, sbackLeft, sbackRight;

    private AnalogInput vfrontLeft, vfrontRight, vbackLeft, vbackRight;

    private AbsoluteAnalogEncoder efrontLeft ,efrontRight, ebackLeft, ebackRight;

    public static double FL_OFFSET  = 2.328 + PI/2, FR_OFFSET = 0.866 + PI/2, BL_OFFSET = 4.552, BR_OFFSET = 0.873 + PI/2;



    public static double hKSQ = 0.3;
    private SquidController hController;

    public SwerveDrivetrain(HardwareMap hwMap) {
        mfrontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        mfrontRight = hwMap.get(DcMotorEx.class, "frontRight");
        mbackLeft = hwMap.get(DcMotorEx.class, "backLeft");
        mbackRight = hwMap.get(DcMotorEx.class, "backRight");

        mfrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        sfrontLeft = hwMap.get(CRServo.class, "sfrontLeft");
        sfrontRight = hwMap.get(CRServo.class, "sfrontRight");
        sbackLeft = hwMap.get(CRServo.class, "sbackLeft");
        sbackRight = hwMap.get(CRServo.class, "sbackRight");

        vfrontLeft = hwMap.get(AnalogInput.class, "efrontLeft");
        vfrontRight = hwMap.get(AnalogInput.class, "efrontRight");
        vbackLeft = hwMap.get(AnalogInput.class, "ebackLeft");
        vbackRight = hwMap.get(AnalogInput.class, "ebackRight");

        efrontLeft = new AbsoluteAnalogEncoder(vfrontLeft, 3.3);
        efrontRight = new AbsoluteAnalogEncoder(vfrontRight, 3.3);
        ebackLeft = new AbsoluteAnalogEncoder(vbackLeft, 3.3);
        ebackRight = new AbsoluteAnalogEncoder(vbackRight, 3.3);

        efrontLeft.zero(FL_OFFSET);
        efrontRight.zero(FR_OFFSET);
        ebackLeft.zero(BL_OFFSET);
        ebackRight.zero(BR_OFFSET);

        frontLeft = new SwerveModule("frontLeft", mfrontLeft, sfrontLeft, efrontLeft);
        frontRight = new SwerveModule("frontRight", mfrontRight, sfrontRight, efrontRight);
        backLeft = new SwerveModule("backLeft", mbackLeft, sbackLeft, ebackLeft);
        backRight = new SwerveModule("backRight", mbackRight, sbackRight, ebackRight);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        modules = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };

        for (SwerveModule m : modules) {
            m.setTargetRotation(0);
        }

        states = new SwerveModule.SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].asState();
        }

        hController = new SquidController(hKSQ);
    }

    public void read() {
        for (SwerveModule module : modules) {
            module.read();
        }
    }

    public void update(Telemetry telemetry) {
        for (SwerveModule module : modules) {
            module.update(telemetry);
        }
    }

    public void setDrivePower(Pose powerVec) {

        double fwd = powerVec.getY();
        double str = powerVec.getX();
        double rcw = powerVec.heading;

        double a = str - rcw * (WB / R);
        double b = str + rcw * (WB / R);
        double c = fwd - rcw * (TW / R);
        double d = fwd + rcw * (TW / R);

        speeds = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
//        speeds = new double[] {0, 0, 0, 0};
        angles = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};

        double max = Math.max(Math.max(speeds[0], speeds[1]), Math.max(speeds[2], speeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) speeds[i] /= max;
        }

        for (int i = 0; i < 4; i++) {
            modules[i].setTargetRotation(angles[i]);
            if (speeds[i] < MIN_WHEEL_POWER[i]) {
                speeds[i] = MIN_WHEEL_POWER[i] * Math.signum(speeds[i]);
            }
            modules[i].setMotorPower(speeds[i]);
        }


    }

    public void getTelemetry(Telemetry telemetry) {
        for (SwerveModule m: modules) {
            telemetry.addData(m.getName() + " target", m.getTargetRotation());
            telemetry.addData(m.getName() + " rotation", m.getModuleRotation());
            telemetry.addData(m.getName() + " servo power", m.getServoPower());
            telemetry.addData(m.getName() + " wheel vel", m.getWheelVelocity());
            telemetry.addData(m.getName() + " flipped", m.flipModifier());
        }
    }

    public void setLocked(boolean locked) {
        this.locked = locked;
    }

    public void setMaintainHeading(boolean maintainHeading) {
        this.maintainHeading = maintainHeading;
    }
}
