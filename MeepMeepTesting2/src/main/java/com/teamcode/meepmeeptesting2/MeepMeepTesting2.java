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

        // beginning position
        Pose2d startPose = new Pose2d(36, 70, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.47254984116122, 60.0, Math.toRadians(132.60000300691226), Math.toRadians(140), 14.02)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(11, 38), Math.toRadians(90)) // PROGRAM PROCESS TO DETECT PIXEL LOCATION
                                .lineToLinearHeading(new Pose2d(11, 47, Math.toRadians(0))) // done detecting custom game piece and placing purple pixel
                                .splineTo(new Vector2d(51, 33), Math.toRadians(0)) // place yellow pixel based on custom game piece position
                                .splineToConstantHeading(new Vector2d(-51, 33), Math.toRadians(0)) // "back" is inaccurate because it relies on relative position, and regular splining takes too long
                                .splineToConstantHeading(new Vector2d(51, 33), Math.toRadians(0)) // place white pixels; end
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}