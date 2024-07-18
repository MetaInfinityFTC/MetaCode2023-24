package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepathonMeeping {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        int cycleOffset = 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(180), 12.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(20, 0, Math.toRadians(90)))
                                .splineTo(new Vector2d(0, 20), Math.toRadians(180))
                                .splineTo(new Vector2d(-20, 0), Math.toRadians(-90))
                                .splineTo(new Vector2d(0,-20), Math.toRadians(0))
                                .splineTo(new Vector2d(20, 0), Math.toRadians(90))
                                //circle
                                .waitSeconds(10)
                                .forward(-10)
                                .lineTo(new Vector2d(0, 20))
                                .lineTo(new Vector2d(-20, -10))
                                .lineTo(new Vector2d(20, -10))
                                //triangle
                                .waitSeconds(10)
                                .lineTo(new Vector2d(20, 0))
                                .splineToSplineHeading(new Pose2d(20,-20,Math.toRadians(0)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-10, 0, Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-10,-20, Math.toRadians(0)), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(20, 0))
                                //infinity
                                .waitSeconds(10)
                                .setTangent(90)
                                .splineToSplineHeading(new Pose2d(0, 20, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-20, 0, Math.toRadians(360)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(0,-20, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(20, 0, Math.toRadians(360)), Math.toRadians(90))
                                //circlespin
                                .waitSeconds(10)
                                .splineToSplineHeading(new Pose2d(20, 20, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(0, 20, Math.toRadians(360)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(20, 0, Math.toRadians(360)), Math.toRadians(90))
                               .build());

        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 12.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(44.75, -45, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(30, -12), 0)
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}