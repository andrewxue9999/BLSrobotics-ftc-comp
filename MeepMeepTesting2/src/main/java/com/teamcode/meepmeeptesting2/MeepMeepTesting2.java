package com.teamcode.meepmeeptesting2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d blueStart1 = new Pose2d(-34, 70, Math.toRadians(-90));
        Pose2d blueStart2 = new Pose2d(34, 70, Math.toRadians(-90));
        Pose2d redStart1 = new Pose2d(-34, -70, Math.toRadians(90));
        Pose2d redStart2 = new Pose2d(34, -70, Math.toRadians(90));

        Vector2d bluePixel1 = new Vector2d(-35, 47);
        Vector2d bluePixel2 = new Vector2d(11, 47);
        Vector2d redPixel1 = new Vector2d(-35, -47);
        Vector2d redPixel2 = new Vector2d(11, -47);

        // beginning position
        Pose2d startPose = new Pose2d(36, 70, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.47254984116122, 60.0, Math.toRadians(132.60000300691226), Math.toRadians(140), 14.02)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
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
                .addEntity(myBot)
                .start();
    }
}