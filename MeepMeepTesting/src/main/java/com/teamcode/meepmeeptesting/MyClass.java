package com.teamcode.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d blueStart1 = new Pose2d(-37, 72, Math.toRadians(-90));
        Pose2d blueStart2 = new Pose2d(37, 72, Math.toRadians(-90));
        Pose2d redStart1 = new Pose2d(-37, -72, Math.toRadians(90));
        Pose2d redStart2 = new Pose2d(37, -72, Math.toRadians(90));

        Vector2d blue1LeftMark = new Vector2d(-48, 34);
        Vector2d blue1CenterMark = new Vector2d(-35, 27);
        Vector2d blue1RightMark = new Vector2d(-25, 34);

//        Vector2d blue2LeftMark = new Vector2d(-48, 29);
//        Vector2d blue2CenterMark = new Vector2d(-48, 29);
//        Vector2d blue2RightMark = new Vector2d(-48, 29);
//
//        Vector2d red1LeftMark = new Vector2d(-48, 29);
//        Vector2d red1CenterMark = new Vector2d(-48, 29);
//        Vector2d red1RightMark = new Vector2d(-48, 29);
//
//        Vector2d red2LeftMark = new Vector2d(-48, 29);
//        Vector2d red2CenterMark = new Vector2d(-48, 29);
//        Vector2d red2RightMark = new Vector2d(-48, 29);


        RoadRunnerBotEntity blue1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.47254984116122, 60.0, Math.toRadians(132.60000300691226), Math.toRadians(140), 14.02)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStart1)
                                // detect y-axis custom object from starting location
                                .setVelConstraint(new TrajectoryVelocityConstraint() {
                                    @Override
                                    public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                                        return 30.0;
                                    }
                                })
                                .lineToConstantHeading(blue1RightMark)
                                .resetVelConstraint()
                                .lineToLinearHeading(new Pose2d(-35,57, Math.toRadians(0))) // start moving towards backboard for yellow pixel
                                .lineToConstantHeading(new Vector2d(10, 57))
                                .splineToConstantHeading(new Vector2d(52, 36), Math.toRadians(0))
                                .waitSeconds(0.75) // drop yellow pixel
                                .splineToConstantHeading(new Vector2d(10, 57), Math.toRadians(0)) // go back for two white pixels
                                .lineToConstantHeading(new Vector2d(-35, 57))
                                .splineToConstantHeading(new Vector2d(-65, 36), Math.toRadians(0))
                                .waitSeconds(0.75) // intake two white pixels
                                .splineToConstantHeading(new Vector2d(-35, 57), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(10, 57))
                                .splineToConstantHeading(new Vector2d(52, 36), Math.toRadians(0))
                                .waitSeconds(0.75) // drop two white pixels on back board
                                .lineTo(new Vector2d(56, 15)) // park in a spot that the other team probably won't park
                                .build()
                );

        RoadRunnerBotEntity blue2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.47254984116122, 60.0, Math.toRadians(132.60000300691226), Math.toRadians(140), 14.02)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStart2)
                                .splineTo(new Vector2d(11, 47), Math.toRadians(-90)) // PROGRAM PROCESS TO DETECT PIXEL LOCATION
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(11, 29)) // place purple pixel
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(11, 47))
                                .lineToSplineHeading(new Pose2d(50, 35, Math.toRadians(0))) // place yellow pixel
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(-10, 61)) // "back" is inaccurate because it relies on relative position, and regular splining takes too long
                                .strafeTo(new Vector2d(-51, 33))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-10, 61), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(51, 33), Math.toRadians(0)) // place white pixels; end
                                .waitSeconds(2)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(blue1)
//                .addEntity(blue2)
                .start();
    }
}