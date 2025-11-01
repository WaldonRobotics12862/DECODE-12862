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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49
                        , 49, Math.toRadians(125)))
                .waitSeconds(2)
                .lineToX(-24)
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-12,24,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-24, 24,Math.toRadians(135)),Math.toRadians(90))
                .waitSeconds(2.5)
                .splineToLinearHeading(new Pose2d(-60,24,Math.toRadians(180)),Math.toRadians(180))
                //.lineToX(-24)
                .build());


        //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -49, Math.toRadians(55))) // Start pose: Y negated, heading = 180° - 125° = 55°
        //        .waitSeconds(2)
        //        .lineToX(-24) // X unchanged
        //        .waitSeconds(2.5)
         //       .setTangent(Math.toRadians(180)) // Tangent mirrored: 0° → 180°
        //        .splineToLinearHeading(new Pose2d(-12, -24, Math.toRadians(90)), Math.toRadians(90)) // Y negated
        //        .lineToY(-48) // Y negated
        //        .setTangent(Math.toRadians(90)) // Tangent mirrored: 270° → 90°
        //        .splineToLinearHeading(new Pose2d(-24, -24, Math.toRadians(45)), Math.toRadians(90)) // Y negated, heading = 180° - 135° = 45°
        //        .waitSeconds(2.5)
        //        .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(0)), Math.toRadians(0)) // Y negated, heading = 180° - 180° = 0°
        //        //.lineToX(-24) // Commented out in original, remains unchanged
        //        .build());
        // Blue Audience Auton
       // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 12, Math.toRadians(180)))  // Blue audience start: facing toward the field (positive y)
                //.lineToX(58)
               // .turn(Math.toRadians(-20))
               // .waitSeconds(3)  // Simulate dropping pixel (comment out hardware code)
               // .splineTo(new Vector2d(36, 32), Math.toRadians(90))
              //  .lineToY(50)
             //   .setTangent(-90)
              //  .splineToLinearHeading(new Pose2d(58,12,Math.toRadians(-200)),Math.toRadians(0))
              //  .build());

        // Red Audience Auton

        //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, -12, Math.toRadians(180)))  // Blue audience start: facing toward the field (positive y)
        //.lineToX(58)
        //.turn(Math.toRadians(20))
        //.waitSeconds(3)  // Simulate dropping pixel (comment out hardware code)
        //.splineTo(new Vector2d(36, -32), Math.toRadians(-90))
        //.lineToY(-50)
        //.setTangent(90)
        //.splineToLinearHeading(new Pose2d(58,-12,Math.toRadians(200)),Math.toRadians(0))
        //build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)  // Centerstage field background
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();  // Launch the visualizer
    }
}