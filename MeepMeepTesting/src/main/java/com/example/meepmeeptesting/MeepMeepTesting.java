package com.example.meepmeeptesting;  // Adjust package as needed

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);  // Window size; adjust as needed

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set your robot's constraints: maxVel (in/s), maxAccel (in/s²), maxAngVel (rad/s), maxAngAccel (rad/s²), trackWidth (in)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 12, Math.toRadians(180)))  // Blue audience start: facing toward the field (positive y)
        //  .lineToX(58)
        //    .turn(Math.toRadians(-20))
        //  .waitSeconds(3)  // Simulate dropping pixel (comment out hardware code)
        //.splineTo(new Vector2d(36, 32), Math.toRadians(90))
        //    .lineToY(50)
        //.setTangent(-90)
        //  .splineToLinearHeading(new Pose2d(58,12,Math.toRadians(-200)),Math.toRadians(0))
        //.build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, -12, Math.toRadians(180)))  // Blue audience start: facing toward the field (positive y)
                .lineToX(58)
                .turn(Math.toRadians(20))
                .waitSeconds(3)  // Simulate dropping pixel (comment out hardware code)
                .splineTo(new Vector2d(36, -32), Math.toRadians(-90))
                .lineToY(-50)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(58,-12,Math.toRadians(200)),Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)  // Centerstage field background
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();  // Launch the visualizer
    }
}