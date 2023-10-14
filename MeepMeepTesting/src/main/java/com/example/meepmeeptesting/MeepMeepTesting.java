package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.47254984116122, 49.47254984116122, Math.toRadians(180), Math.toRadians(180), 14.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -35, Math.toRadians(0)))
                                // Simple auto test



                                // The robot can move in 4 different directions independently thanks to our mecanum wheels/
                                // We can move forwards, backwards, strafe left, and strafe right
                                // Drive forward 40 inches
//                                .forward(40)

                                // Drive backward 40 inches
//                                .back(40)

                                // Strafe left 40 inches
//                                .strafeLeft(40)

                                // Strafe right 40 inches
//                                .strafeRight(40)

                                // We can also preform simple turns or wait a given amount of time
                                // Turn 90 degrees counter-clockwise
//                                .turn(Math.toRadians(90))

                                // Wait 3 seconds
//                                .waitSeconds(3)

                                // Roadrunner also carries some more complex ways to get the robot to move somewhere
                                // Before, we get into that, let's talk a bit about heading and how positioning works
                                // The robot understands it's position through a coordinate grid with a X and Y axis
                                // Heading is the direction the robot faces, where 0 degrees is east, 90 degrees is north, and so forth
                                // Our robot is able to move in a direction while facing a different direction due to special mecanum wheels.

                                // Now that we understand the coordinate plane and heading, we need to learn about the two ways that roadrunner can store positions
                                // A Vector2d is a 2d vector which contains two values, a X and Y coordinate.
                                    // Create a vector at coordinate (10, -5)
                                    // Vector2d myVector = new Vector2d(10, -5);
                                // A Pose2d represents the position of the robot AND its heading (or the direction it faces)
                                    // Create a pose at coordinate (10, -5) facing 90 degrees
                                    // Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
                                // Any function that RESULTS IN A HEADING CHANGE will use a Pose2d
                                // Any function that HAS THE SAME HEADING IN THE START AND END will use a Vector2d

                                // Roadrunner has multiple functions to get a robot to a certain Pose2d or Vector2d (remember that a Pose or Vector represents a position in space)
                                // Line to forces the robot to move to a specified VECTOR.
                                // The robot will maintain it's heading (direction) which is starts out with.
                                // lineTo is functionally the same as strafeTo and lineToConstantHeading
//                                .lineTo(new Vector2d(40, 40))

                                // lineToLinearHeading does exactly as it's name suggests.
                                // The robot will move to the POSE while linearly changing between the start heading and end heading
                                // Here, the robot moves to the new POSE while changing it's direction
//                                .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))

                                // lineToSplineHeading is similar to lineToLinearHeading
                                // The robot will move to the POSE while SPLINE interpolating between the start and end heading
                                // What is spline interpolating?
                                // Spline interpolating (at it's most fundamental level) is changing between two values through a parabolic or cubic function
                                // Essentially, the action is eased-in and eased-out, resulting a smoother movement.
//                                .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)))

                                // splineTo is slightly different than lineTo
                                // While lineTo has the robot move in a straight line to the specified VECTOR, splineTo moves in a spline
                                // In simple terms, a spline is a mathematical function, generally cubic.
                                // Similar to lineTo, the robot will start and end at the same heading (direction)
                                // However, unlike lineTo, the robot WILL TURN and avoid strafing (moving left or right) as much as possible
                                // splineTo requires a Pose2d and end tangent (in radians). The end tangent affects the path of the robot
//                                .splineTo(new Vector2d(40, 40), Math.toRadians(0))

                                // Unlike lineTo and lineToConstantHeading, splineTo and splineToConstantHeading act differently
                                // While splineTo permits the robot the change heading while moving, splineToConstantHeading does not
                                // This means that the robot will be facing one direction throughout it's path
//                                .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))

                                // splineToLinearHeading acts similar to lineToLinearHeading
                                // As the robot moves to the Vector2d on a spline path, it changes its heading linearly
//                                .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))

                                // splineToSplineHeading acts similar to splineToSplineHeading
                                // As the robot moves to the Pose2d on a spline path, it interpolates the heading through a spline
//                                .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))



                                // A command essentially telling the robot to follow the commands stated above
                                .build()
                );

        Image img = null;
        // Pathname requires the path to the file of the background image (png)
        try { img = ImageIO.read(new File("C:\\Users\\Public\\Public Pictures\\centerstage-bg.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}