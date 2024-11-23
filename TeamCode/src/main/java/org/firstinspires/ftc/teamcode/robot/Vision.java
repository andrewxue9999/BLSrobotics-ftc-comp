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
    private double tx; // how off we are horizontally, degrees
    private double ty; // how off we are vertically, degrees
    private Pose3D botPose; // (0, 0, 0) is center of field floor

    public void init(@NonNull HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.pipelineSwitch(0); // assuming apriltag pipeline = 0 when hardware is configured

        limelight.setPollRateHz(100);
        limelight.start();
    }

    // find how off we are from a tag, angle-wise.
    public double[] getTagData() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            botPose = result.getBotpose();
            tx = result.getTx();
            ty = result.getTy();

            // result.getTa(): how big the target looks (0-100% of image)
            return new double[]{tx, ty};
        }
//          String t = getTelemetrySpecific("tx", Double.toString(result.getTx())) + getTelemetrySpecific("ty", Double.toString(result.getTy())) + getTelemetrySpecific("Botpose", botpose.toString());return result;
        return null;
    }


    // get current robot position with apriltags (only call if apriltag in view)
    // "MegaTag 1"
    public double[] getPosData() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            botPose = result.getBotpose();
            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;

                return new double[]{x, y};
            }
        }
        return null;
    }

    // could be useful for incorporating time delay/lag into detecting our true robot position
    public long getOffset() {
        return limelight.getLatestResult().getStaleness();
    }

    // take a snapshot, puts it onto the web interface's Input Tb
    // can use the snapshot as "image source" to tune pipelines
    public String takeSnapshot() {
        limelight.captureSnapshot("auto_pov_10s");
        return "Snapshot taken"; // return string to be incorporated into telemetry via Teleop.java
    }

    public String clearSnapshots() {
        limelight.deleteSnapshots();
        return "Snapshots deleted";
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

/*
MegaTag 2 is like MegaTag 1, but it fuses your IMU data for increased accuracy:

// First, tell Limelight which way your robot is facing
double robotYaw = imu.getAngularOrientation().firstAngle;
limelight.updateRobotOrientation(robotYaw);
if (result != null && result.isValid()) {
    Pose3D botpose_mt2 = result.getBotpose_MT2();
    if (botpose_mt2 != null) {
        double x = botpose_mt2.getPosition().x;
        double y = botpose_mt2.getPosition().y;
        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
    }
}
 */