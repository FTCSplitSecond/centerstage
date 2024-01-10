package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class AlliancePosition{
    LEFT,
    RIGHT
}
class GoToBlueTopSpike(val robot: Robot, val alliancePosition: AlliancePosition): CommandBase() {
    val currentPose = robot.driveBase.poseEstimate()
    val sequenceBuilder = robot.driveBase.trajectorySequenceBuilder(currentPose)
    val endPose = when (alliancePosition){
        AlliancePosition.LEFT -> Pose2d(0.0,0.0,0.0)
        AlliancePosition.RIGHT -> Pose2d(-12.8, -35.65, 0.0)

    }

    /*val trajectory = when (alliancePosition){
        AlliancePosition.LEFT -> sb.st
    }*/
}