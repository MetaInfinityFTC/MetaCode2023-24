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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 12.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14.75, -61.5, Math.toRadians(-90)))
                                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                                    //extend to midspike
                                })
                                .lineToSplineHeading(new Pose2d(45, -32, Math.toRadians(-190)))
                                .addTemporalMarker(() -> {
                                    //place yellow & purple
                                })
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(20, -35, Math.toRadians(-180)), Math.toRadians(180))
                                .lineTo(new Vector2d(-15, -35))
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //grab
                                })
                                .lineTo(new Vector2d(20, -35))
                                .splineToSplineHeading(new Pose2d(45, -30, Math.toRadians(-180)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //drop
                                })
                                .splineToSplineHeading(new Pose2d(20, -35, Math.toRadians(-180)), Math.toRadians(180))
                                .lineTo(new Vector2d(-15, -35))
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //grab
                                })
                                .lineTo(new Vector2d(20, -35))
                                .splineToSplineHeading(new Pose2d(45, -30, Math.toRadians(-180)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //drop
                                })


                                .setTangent(Math.toRadians(135))
                                .splineToConstantHeading(new Vector2d(20, -10.5), Math.toRadians(180))
                                .lineTo(new Vector2d(-15, -10.5))
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //grab
                                })
                                .lineTo(new Vector2d(20, -10.5))
                                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //drop
                                })


                                .setTangent(Math.toRadians(135))
                                .splineToConstantHeading(new Vector2d(20, -10.5), Math.toRadians(180))
                                .lineTo(new Vector2d(-15, -10.5))
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //grab
                                })
                                .lineTo(new Vector2d(20, -10.5))
                                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //drop
                                })
                                .lineToSplineHeading(new Pose2d(45, -57, Math.toRadians(-180)))
                                .build());
                ;

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