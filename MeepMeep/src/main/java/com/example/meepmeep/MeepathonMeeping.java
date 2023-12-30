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
                .setConstraints(75, 60, Math.toRadians(214.78926857142858), Math.toRadians(214.78926857142858), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14.75, 61.9, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(49, 39, Math.toRadians(0)))
                                .waitSeconds(0.2)
                                .strafeRight(12)
                                .waitSeconds(0.2)
                                .strafeLeft(10)
                                .lineToSplineHeading(new Pose2d(10, 36, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(-(61-35), 36, Math.toRadians(0)), Math.toRadians(180))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(49, 38))
                                .waitSeconds(0.2)
                                .lineToSplineHeading(new Pose2d(-(61-30), 36, Math.toRadians(0)))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(49, 38))
                                .waitSeconds(0.2)
                                .setTangent(Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(40, 25))
                                .splineToConstantHeading(new Vector2d(-(61-30), 11.75), Math.toRadians(180))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(25, 12))
                                .splineToConstantHeading(new Vector2d(49, 36), Math.toRadians(25))
                                .waitSeconds(0.2)
                                .setTangent(Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(40, 25))
                                .splineToConstantHeading(new Vector2d(-(61-30), 11.75), Math.toRadians(180))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(21, 12))
                                .splineToConstantHeading(new Vector2d(49, 30), Math.toRadians(25))
                                .waitSeconds(0.2)
                                .build());
                ;

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}