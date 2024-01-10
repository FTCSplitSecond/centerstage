package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.drone_launcher.Commands.SetDroneForInit
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

@Autonomous
class BlueParkAuto: CommandOpMode() {
    override fun initialize() {
//        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)
//
//        val startPose = Pose2d(0.0, 0.0, 0.0)
//        robot.driveBase.poseEstimate = startPose
//
//        val autoCommands = SequentialCommandGroup(
//            MoveToPosition(robot, Pose2d(6.0,30.0 , 0.0)),
////            MoveToPosition(robot, Pose2d(12.0, 12.0, PI/2.0))
//        )
//
//        schedule(autoCommands)
//        SetDroneForInit(robot.droneLauncher)

    }
}