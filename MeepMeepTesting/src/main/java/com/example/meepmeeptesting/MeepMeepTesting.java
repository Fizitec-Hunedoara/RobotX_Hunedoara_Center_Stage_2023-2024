package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL, DriveConstants.TRACK_WIDTH)
//                // Option: Set theme. Default = ColorSchemeRedDark()
//                .setColorScheme(new ColorSchemeRedDark())
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(14.783464, 62.73622, Math.toRadians(270)))
////                                .lineTo(new Vector2d(14, 33))
////                                .lineToLinearHeading(new Pose2d(new Vector2d(52, 34), Math.toRadians(180)))
////                                .splineTo(new Vector2d(10,60),Math.toRadians(180))
////                                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 60), Math.toRadians(180)))
////                                .splineTo(new Vector2d(-52,55),Math.toRadians(235))
////                                .setReversed(true)
////                                .splineTo(new Vector2d(-30,60),Math.toRadians(0))
////                                .lineToSplineHeading(new Pose2d(new Vector2d(10,60),Math.toRadians(180)))
////                                .splineTo(new Vector2d(52,40),Math.toRadians(0))
//                                .splineTo(new Vector2d(0,0),Math.toRadians(180))
//                                .build()
//                );
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL, DriveConstants.TRACK_WIDTH)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14.783464, 62.73622, Math.toRadians(270)))
                                .lineTo(new Vector2d(14, 33))
                                .lineToLinearHeading(new Pose2d(new Vector2d(52, 34), Math.toRadians(180)))
                                .splineTo(new Vector2d(10,59),Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 59), Math.toRadians(180)))
                                .splineTo(new Vector2d(-52,45),Math.toRadians(235))
                                .setReversed(true)
                                .splineTo(new Vector2d(-30,58),Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(new Vector2d(10,58),Math.toRadians(180)))
                                .splineTo(new Vector2d(52,40),Math.toRadians(0))
                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
