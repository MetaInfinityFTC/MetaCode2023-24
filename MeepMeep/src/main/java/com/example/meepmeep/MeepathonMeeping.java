package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepathonMeeping {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52, 52, Math.toRadians(214.78926857142858), Math.toRadians(214.78926857142858), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                                .splineTo(new Vector2d(10, -38), Math.toRadians(135))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(50,-29, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(180))
                                .forward(78)
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(20, -12))
                                .splineToConstantHeading(new Vector2d(50, -28), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(180))
                                .forward(78)
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(20, -12))
                                .splineToConstantHeading(new Vector2d(50, -28), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(180))
                                .forward(78)
                                .waitSeconds(0.3)
                                .strafeLeft(12)
                                .waitSeconds(0.3)
                                .strafeRight(3)
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, -28), Math.toRadians(0))
                                .waitSeconds(0.5)
                                /*.splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(180))
                                .forward(50)
                                .splineToConstantHeading(new Vector2d(-58, -24, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(0.5)
                                .back(10)
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Pose2d(50, -28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)*/
                                .strafeRight(5)
                                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}