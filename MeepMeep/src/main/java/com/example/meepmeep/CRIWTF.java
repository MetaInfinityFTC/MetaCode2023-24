package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class CRIWTF {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(180), 12.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 58, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                                })
                                // .lineToSplineHeading(new Pose2d(-42.5, 11, Math.toRadians(90)))
                                .back(5)
                                .turn(Math.toRadians(25))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .back(5)
                                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(-180)))
                                .lineTo(new Vector2d(20, 47))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                })
                                .splineTo(new Vector2d(43, 40), Math.toRadians(0))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(1)
                                .forward(2)
                                .addTemporalMarker(() -> {
                                })
                                .forward(4)
                                .lineTo(new Vector2d(42, 10))
                                .build());
        RoadRunnerBotEntity blueMiddle = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(180), 12.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 58, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                                })
                                .back(5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                })
                                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                })
                                .lineTo(new Vector2d(10, 47))
                                .lineToSplineHeading(new Pose2d(20, 47, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                })
                                .splineTo(new Vector2d(43, 35), Math.toRadians(0))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(0.5)
                                .forward(4)
                                .addTemporalMarker(() -> {
                                })
                                .lineTo(new Vector2d(42, 10))
                                .build());

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(180), 12.55)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 58, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                })
                                .back(5)
                                .turn(Math.toRadians(-25))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(-180)))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                })
                                .lineTo(new Vector2d(20, 47))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .splineTo(new Vector2d(43, 30), Math.toRadians(0))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(0.5)
                                .forward(4)
                                .addTemporalMarker(() -> {
                                })
                                .lineTo(new Vector2d(42, 10))
                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueLeft)
                .addEntity(blueMiddle)
                .addEntity(blueRight)
                .start();
    }
}