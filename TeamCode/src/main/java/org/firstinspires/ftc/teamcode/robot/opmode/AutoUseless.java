package org.firstinspires.ftc.teamcode.robot.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.hardware.swerve.AutoSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.Claw;
import org.firstinspires.ftc.teamcode.robot.hardware.Pivot;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.roadrunner.main.trajectorysequence.TrajectorySequence;

@Autonomous(name = "(use this fr) Autonomous (just park)")
public class AutoUseless extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public final double TRACKWIDTH = 12.6378;
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
    public AutoSwerveDrivetrain drivetrain;

    private final Pose2d DEFAULT_CENTER = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {
        drivetrain = new AutoSwerveDrivetrain(hardwareMap);

//        drivetrain.setPoseEstimate(DEFAULT_CENTER); // change as see fit; search up coordinate map for this year's challenge.

        // reference documentation or Centerstage code (ex. org/firstinspires/ftc/teamcode/CenterStage/Auto/AutoBlue1.java)
        TrajectorySequence path1 = AutoSwerveDrivetrain.trajectorySequenceBuilder(new Pose2d(1, 0, 0.5))
                .setVelConstraint(AutoSwerveDrivetrain.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)) // so it doesn't just go too fast
//                .lineToConstantHeading(new Vector2d(5, 0)) // random location
                .setVelConstraint(AutoSwerveDrivetrain.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .waitSeconds(25)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drivetrain.followTrajectorySequence(path1);
    }
}