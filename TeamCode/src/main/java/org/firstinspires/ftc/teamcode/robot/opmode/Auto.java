//package org.firstinspires.ftc.teamcode.robot.opmode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robot.hardware.Claw;
//import org.firstinspires.ftc.teamcode.robot.hardware.Pivot;
//import org.firstinspires.ftc.teamcode.robot.hardware.swerve.SwerveDrivetrain;
//import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.drive.SwerveDrive;
//
//@Autonomous(name = "(use this fr) Autonomous")
//public class Auto extends LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//    public final double TRACKWIDTH = 12.6378;
//    public final double WHEELBASE = 12.6378;
//    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
//    private SlewRateLimiter fw;
//    private SlewRateLimiter str;
//    public static double fw_r = 4;
//    public static double str_r = 4;
//    public SwerveDrivetrain drivetrain;
//    public Pivot pivot;
//    private Claw claw;
//
//    @Override
//    public void runOpMode() {
//
//        drivetrain = new SwerveDrivetrain();
//        drivetrain.init(hardwareMap);
//
//        fw = new SlewRateLimiter(fw_r);
//        str = new SlewRateLimiter(str_r);
//
//        pivot = new Pivot();
//        pivot.init(hardwareMap, "pivot", "poop", "extendo");
//
//        claw = new Claw();
//        claw.init(hardwareMap, "claw");
//
////
////        drive.setPoseEstimate(redStart2);
////
////
////        TrajectorySequence test = drive.trajectorySequenceBuilder(redStart2)
////                // detect y-axis custom object from starting location
////                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)) // so it doesn't just go too fast
////                .lineToConstantHeading(desiredMark) // place white pixel on mark
////                .waitSeconds(0.25)
////                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
////
////                .lineToLinearHeading(new Pose2d(35,-57, Math.toRadians(0))) // start moving towards backboard for yellow pixel
////                .addDisplacementMarker(() -> {
////                    armServo1.setPosition(0.1);
////                    armServo2.setPosition(0.1);
////                })
////
////
////
////                .splineToConstantHeading(aprilTagLocation, Math.toRadians(0)) // arrive at backboard for yellow pixel
////                .addSpatialMarker(new Vector2d(50, -36), () -> {
////                    liftMotor1.setPower(1.0); // lift before arm for enough clearance
////                    liftMotor2.setPower(1.0);
////
////                    armServo1.setPosition(1.0);
////                    armServo2.setPosition(1.0);
////                })
////                .waitSeconds(0.35) // drop yellow pixel
////                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
////                    armServo1.setPosition(0.1); // arm before lift for enough clearance on the way back down
////                    armServo2.setPosition(0.1);
////
////                    liftMotor1.setPower(0.0);
////                    liftMotor2.setPower(0.0);
////                })
////
////                .splineToConstantHeading(new Vector2d(10, -57), Math.toRadians(0)) // go back for two white pixels
////
////                .lineToConstantHeading(new Vector2d(-35, -57))
////                .splineToLinearHeading(new Pose2d(-68, -36, Math.toRadians(0)), Math.toRadians(179))
////                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
////                    intakeMotor.setPower(1);
////                })
////                .waitSeconds(0.75) // intake two white pixels
////                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
////                    intakeMotor.setPower(0);
////                })
////                .splineToLinearHeading(new Pose2d(-35, -57, Math.toRadians(179)), Math.toRadians(0))
////                .lineToConstantHeading(new Vector2d(10, -57))
////                .splineToConstantHeading(aprilTagLocation, Math.toRadians(0))
////                .addSpatialMarker(new Vector2d(50, -36), () -> {
////                    liftMotor1.setPower(1.0);
////                    liftMotor2.setPower(1.0);
////
////                    armServo1.setPosition(1.0);
////                    armServo2.setPosition(1.0);
////
////                })
////                .waitSeconds(0.35) // drop two white pixels on backboard
////                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
////                    armServo1.setPosition(0.1);
////                    armServo2.setPosition(0.1);
////
////                    liftMotor1.setPower(0.0);
////                    liftMotor2.setPower(0.0);
////                })
////                .lineTo(new Vector2d(56, -15)) // park in a spot that the other team probably won't park
////                .build();
////
//        waitForStart();
////
//        if (isStopRequested()) return;
////
////        drive.followTrajectorySequence(test);
//    }
//}