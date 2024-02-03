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
                .setConstraints(60, 60, Math.toRadians(214.78926857142858), Math.toRadians(214.78926857142858), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33, -58, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(-33, -12, Math.toRadians(-90)))
                                //claw goes out here
                                .turn(Math.toRadians(25))
                                //add claw drop
                                .lineToSplineHeading(new Pose2d(-20, -12, Math.toRadians(-180)))
                                //claw in
                                //.waitSeconds(20) -> timer for other team to finish auto
                                .lineTo(new Vector2d(20, -12))
                                //depo arm out now
                                .splineTo(new Vector2d(45, -30), Math.toRadians(0))
                                //place yellow pixel, depo arm in, etc.
                                .strafeRight(15)
                                .build());
                ;

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}