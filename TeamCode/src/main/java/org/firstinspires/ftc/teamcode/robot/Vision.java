package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.dashboard.config.Config;

public class Vision {

    private Limelight3A limelight;
    private double tx;
    private double ty;
    private Pose3D botPose;

    public void init(@NonNull HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.pipelineSwitch(0); // assuming apriltag pipeline = 0 when hardware is configured
        limelight.start();
    }

    public LLResult getData() {
        LLResult result = limelight.getLatestResult();

        botPose = result.getBotpose();
        tx = result.getTx();
        ty = result.getTy();

//          String t = getTelemetrySpecific("tx", Double.toString(result.getTx())) + getTelemetrySpecific("ty", Double.toString(result.getTy())) + getTelemetrySpecific("Botpose", botpose.toString());return result;

        return result;
    }

    // general case, for debugging
    public String getTelemetry() {
        return String.format("tx: %s, ty: %s, botPose: %s", tx, ty, botPose);
    }

    // for specific stuff
    public String getTelemetrySpecific(String title, String description) {
        return title + ": " + description + "; ";
    }
}


/*
    From documentation, that uses the IMU to optimize apriltag detection:

    For maximum 3D localization accuracy, call updateRobotOrientation() and use getBotPose_MT2().
    MegaTag2 is an IMU-Fused robot localizer that utilizes the imu to solve the ambiguity problem
    which is fundamental to all planar targets such as AprilTags.


     while (opModeIsActive()) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
 */