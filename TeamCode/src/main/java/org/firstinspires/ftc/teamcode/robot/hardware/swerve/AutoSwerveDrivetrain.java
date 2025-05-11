//// for autonomous use only
//
//package org.firstinspires.ftc.teamcode.robot.hardware.swerve;
//
//import com.acmerobotics.roadrunner.drive.SwerveDrive;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.MAX_ACCEL;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.MAX_ANG_ACCEL;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.MAX_ANG_VEL;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.MAX_VEL;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.MOTOR_VELO_PID;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.RUN_USING_ENCODER;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.TRACK_WIDTH;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.kA;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.kStatic;
//import static org.firstinspires.ftc.teamcode.roadrunner.main.drive.DriveConstants.kV;
//
//import androidx.annotation.GuardedBy;
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.drive.DriveSignal;
//import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
//import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.roadrunner.main.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.roadrunner.main.trajectorysequence.TrajectorySequenceBuilder;
//import org.firstinspires.ftc.teamcode.roadrunner.main.trajectorysequence.TrajectorySequenceRunner;
//import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//@Config
//public class AutoSwerveDrivetrain extends SwerveDrive {
//
//    public SwerveModule leftFront, leftBack, rightFront, rightBack;
//    private DcMotorEx mrightFront;
//    private DcMotorEx mleftFront;
//    private DcMotorEx mleftBack;
//    private DcMotorEx mrightBack;
//
//    private CRServo srightFront;
//    private CRServo sleftFront;
//    private CRServo sleftBack;
//    private CRServo srightBack;
//
//    private AnalogInput erightFront;
//    private AnalogInput eleftFront;
//    private AnalogInput eleftBack;
//    private AnalogInput erightBack;
//    private static double analogRange = 3.3;
//
//    public static final double E_RIGHT_FRONT_OFFSET = -Math.PI/2 + 5.0932; //2.0449; // RADianz
//    public static final double E_LEFT_FRONT_OFFSET = -Math.PI/2 + 5.1198; //1.14424;
//    public static final double E_LEFT_BACK_OFFSET = -Math.PI/2 + 4.8342; // 1.487;
//    public static final double E_RIGHT_BACK_OFFSET = -Math.PI/2 + 5.3826; //3.9565;
//
//
//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(4, 0, 0);
//        public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);
//
//        public static double FL_STATIC = 0.2;
//        public static double FR_STATIC = 0.2;
//        public static double RL_STATIC = 0.2;
//        public static double RR_STATIC = 0.2;
//
//        public static double VX_WEIGHT = 1;
//        public static double VY_WEIGHT = 1;
//        public static double OMEGA_WEIGHT = 1;
//
//        public static int MAX_PARALLEL_COMMANDS = 8;
//
//        private final TrajectorySequenceRunner trajectorySequenceRunner;
//
//        private final TrajectoryVelocityConstraint velocityConstraint;
//
//        private final TrajectoryAccelerationConstraint accelConstraint;
//
//        private final TrajectoryFollower follower;
//
//        public SwerveModule leftFrontModule, leftRearModule, rightRearModule, rightFrontModule;
//        public List<SwerveModule> modules;
//
//        private final VoltageSensor batteryVoltageSensor;
//
//        public Thread imuThread;
//
//        private final Object IMULock = new Object();
//        private double imuAngle = 0;
//        private double imuAngleVelocity = 0;
//        @GuardedBy("IMULock")
//        private BNO055IMU imu;
//
//        public AutoSwerveDrivetrain(HardwareMap hardwareMap) {
//            super(kV, kA, kStatic, TRACK_WIDTH);
//
//            velocityConstraint = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
//            accelConstraint = getAccelerationConstraint(MAX_ACCEL);
//
//            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
//                    new Pose2d(0.5, 0.5, Math.toRadians(5)), 0);
//
//            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//
////            synchronized (IMULock) {
////                imu = hardwareMap.get(BNO055IMU.class, "imu");
////                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
////                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
////                imu.initialize(parameters);
////            }
//
//            mleftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
//            mleftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
//            mrightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
//            mrightBack = hardwareMap.get(DcMotorEx.class, "backRight");
//
//            mleftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//            mleftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            mleftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            mleftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            mrightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            mrightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            sleftFront = hardwareMap.get(CRServo.class, "sfrontLeft");
//            sleftBack = hardwareMap.get(CRServo.class, "sbackLeft");
//            srightFront = hardwareMap.get(CRServo.class, "sfrontRight");
//            srightBack = hardwareMap.get(CRServo.class, "sbackRight");
//
//            eleftFront = hardwareMap.get(AnalogInput.class, "efrontLeft");
//            eleftBack = hardwareMap.get(AnalogInput.class, "ebackLeft");
//            erightFront = hardwareMap.get(AnalogInput.class, "efrontRight");
//            erightBack = hardwareMap.get(AnalogInput.class, "ebackRight");
//
//
//            leftFrontModule = new SwerveModule(mleftBack, sleftBack, new AbsoluteAnalogEncoder(eleftBack, analogRange).zero(E_LEFT_BACK_OFFSET));
//            leftRearModule = new SwerveModule(mleftFront, sleftFront, new AbsoluteAnalogEncoder(eleftFront, analogRange).zero(E_LEFT_FRONT_OFFSET));
//            rightFrontModule = new SwerveModule(mrightFront, srightFront, new AbsoluteAnalogEncoder(erightFront, analogRange).zero(E_RIGHT_FRONT_OFFSET));
//            rightRearModule = new SwerveModule(mrightBack, srightBack, new AbsoluteAnalogEncoder(erightBack, analogRange).zero(E_RIGHT_BACK_OFFSET));
//
//            modules = Arrays.asList(leftFrontModule, leftRearModule, rightRearModule, rightFrontModule);
//
//
//            if (RUN_USING_ENCODER) {
//                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//
//            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//            if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
//                setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
//            }
//
//            trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID); // include extra omitted parameters ? (reference official RR github for TrajectorySequenceRunner)
//        }
//
////        public void startIMUThread(LinearOpMode opMode) {
////            imuThread = new Thread(() -> {
////                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
////                    synchronized (IMULock) {
////                        imuAngle = imu.getAngularOrientation().firstAngle;
////                        imuAngleVelocity = -imu.getAngularVelocity().xRotationRate;
////                    }
////                }
////            });
////            imuThread.start();
////        }
//
//        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
//            return new TrajectoryBuilder(startPose, velocityConstraint, accelConstraint);
//        }
//
//        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
//            return new TrajectoryBuilder(startPose, reversed, velocityConstraint, accelConstraint);
//        }
//
//        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
//            return new TrajectoryBuilder(startPose, startHeading, velocityConstraint, accelConstraint);
//        }
//
//        public static TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
//            return new TrajectorySequenceBuilder(
//                    startPose,
//                    getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL),
//                    MAX_ANG_VEL, MAX_ANG_ACCEL
//            );
//        }
//        public static TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose, double startHeading) {
//            return new TrajectorySequenceBuilder(
//                    startPose,
//                    startHeading,
//                    getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL),
//                    MAX_ANG_VEL, MAX_ANG_ACCEL
//            );
//        }
//        public void turnAsync(double angle) {
//            trajectorySequenceRunner.followTrajectorySequenceAsync(
//                    trajectorySequenceBuilder(getPoseEstimate())
//                            .turn(angle)
//                            .build()
//            );
//        }
//
//        public void turn(double angle) {
//            turnAsync(angle);
//            waitForIdle();
//        }
//
//        public void followTrajectoryAsync(Trajectory trajectory) {
//            trajectorySequenceRunner.followTrajectorySequenceAsync(
//                    trajectorySequenceBuilder(trajectory.start())
//                            .addTrajectory(trajectory)
//                            .build()
//            );
//        }
//
//        public void followTrajectory(Trajectory trajectory) {
//            followTrajectoryAsync(trajectory);
//            waitForIdle();
//        }
//
//        public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
//            trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
//        }
//
//        public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
//            followTrajectorySequenceAsync(trajectorySequence);
//            waitForIdle();
//        }
//
//        public Pose2d getLastError() {
//            return trajectorySequenceRunner.getLastPoseError();
//        }
//
//        public void updateModules(){
//            for (SwerveModule m : modules) m.update();
//
//
//        }
//        public void update() {
//            updateModules();
//            updatePoseEstimate();
//            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
//            if (signal != null) setDriveSignal(signal);
//
//        }
//
//        public void waitForIdle() {
//            while (!Thread.currentThread().isInterrupted() && isBusy())
//                update();
//
//        }
//
//        public boolean isBusy() {
//            return trajectorySequenceRunner.isBusy();
//        }
//
//        public void setMode(DcMotor.RunMode runMode) {
//            for (SwerveModule m : modules) m.setMode(runMode);
//        }
//
//        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
//            for (SwerveModule m : modules) m.setZeroPowerBehavior(zeroPowerBehavior);
//
//        }
//
//        public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
//            PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
//                    coefficients.p, coefficients.i, coefficients.d,
//                    coefficients.f * 12 / batteryVoltageSensor.getVoltage()
//            );
//
//            for (SwerveModule m : modules) m.setPIDFCoefficients(runMode, compensatedCoefficients);
//        }
//
//        public void setWeightedDrivePower(Pose2d drivePower) {
//            Pose2d vel = drivePower;
//
//            if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
//                    + Math.abs(drivePower.getHeading()) > 1) {
//                // re-normalize the powers according to the weights
//                double denom = VX_WEIGHT * Math.abs(drivePower.getX())
//                        + VY_WEIGHT * Math.abs(drivePower.getY())
//                        + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
//
//                vel = new Pose2d(
//                        VX_WEIGHT * drivePower.getX(),
//                        VY_WEIGHT * drivePower.getY(),
//                        OMEGA_WEIGHT * drivePower.getHeading()
//                ).div(denom);
//            }
//
//            setDrivePower(vel);
//        }
//
//        @NonNull
//        @Override
//        public List<Double> getWheelPositions() {
//            List<Double> wheelPositions = new ArrayList<>();
//            for (SwerveModule m : modules) wheelPositions.add(m.getWheelPosition());
//            return wheelPositions;
//        }
//
//        @Override
//        public List<Double> getWheelVelocities() {
//            List<Double> wheelVelocities = new ArrayList<>();
//            for (SwerveModule m : modules) wheelVelocities.add(m.getWheelVelocity());
//            return wheelVelocities;
//        }
//
//        @Override
//        public void setMotorPowers(double v, double v1, double v2, double v3) {
//            leftFrontModule.setMotorPower(v);
//            leftRearModule.setMotorPower(v1);
//            rightRearModule.setMotorPower(v2);
//            rightFrontModule.setMotorPower(v3);
//        }
//
//        @Override
//        public double getRawExternalHeading() {
//            return imuAngle;
//        }
//
//        @Override
//        public Double getExternalHeadingVelocity() {
//            // To work around an SDK bug, use -zRotationRate in place of xRotationRate
//            // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
//            // expected). This bug does NOT affect orientation.
//            //
//            // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
//            return imuAngleVelocity;
//
//        }
//
//        public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
//            return new MinVelocityConstraint(Arrays.asList(
//                    new AngularVelocityConstraint(maxAngularVel),
//                    new MecanumVelocityConstraint(maxVel, trackWidth)
//            ));
//        }
//
//        public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
//            return new ProfileAccelerationConstraint(maxAccel);
//        }
//
//        @NonNull
//        @Override
//        public List<Double> getModuleOrientations() {
//            List<Double> moduleOrientations = new ArrayList<>();
//            for (SwerveModule m : modules) moduleOrientations.add(m.getModuleRotation());
//
//            return moduleOrientations;
//        }
//
//
//        @Override
//        public void setModuleOrientations(double v, double v1, double v2, double v3) {
//            leftFrontModule.setTargetRotation(v);
//            leftRearModule.setTargetRotation(v1);
//            rightRearModule.setTargetRotation(v2);
//            rightFrontModule.setTargetRotation(v3);
//        }
//
////        1
//
//}
