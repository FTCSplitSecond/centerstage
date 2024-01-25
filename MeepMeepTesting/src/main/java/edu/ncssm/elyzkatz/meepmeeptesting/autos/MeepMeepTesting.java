package edu.ncssm.elyzkatz.meepmeeptesting.autos;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        DefaultBotBuilder botBuilder = new DefaultBotBuilder(meepMeep);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> blueFarRight(drive)
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static TrajectorySequence blueFarCenter(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-32.0, 62.0, PI / 2))
                .lineTo(new Vector2d(-36.0, 62.0))
                .lineTo(new Vector2d(-36.0, 8.0))
                .lineTo(new Vector2d(24.0, 8.0))
                .lineToLinearHeading(new Pose2d(47.0, 30.0, -PI))
                .lineTo(new Vector2d(47.0, 60.0))
                .build();
    }
    private static TrajectorySequence blueFarRight(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-32.0, 62.0, PI / 2))
                .lineToLinearHeading(new Pose2d(-36.0, 62.0, -PI))
                .lineTo(new Vector2d(-36.0, 8.0))
                .lineTo(new Vector2d(24.0, 8.0))
                .lineTo(new Vector2d(33.0, 24.0))
                .lineTo(new Vector2d(42.0, 24.0))
                .lineTo(new Vector2d(42.0, 6.0))

//                .lineTo(new Vector2d(-32.0, 8.0))
//                .lineTo(new Vector2d(24.0, 8.0))
//                .lineToLinearHeading(new Pose2d(47.0, 30.0, -PI))
//                .lineTo(new Vector2d(47.0, 60.0))
                .build();
    }
}