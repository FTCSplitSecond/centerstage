package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.DroneLaunch.Commands.SetDroneForInit
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.robot.commands.MoveToDeposit
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPlace
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPosition
import org.firstinspires.ftc.teamcode.robot.commands.MoveToTravel
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.PI

@Autonomous
class BlueParkAuto: CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)

        val startPose = Pose2d(0.0, 0.0, 0.0)
        robot.driveBase.poseEstimate = startPose

        val autoCommands = SequentialCommandGroup(
            MoveToPosition(robot, Pose2d(24.0, 36.0, -PI/2)),

        )
//            MoveToPosition(robot, Pose2d(12.0, 12.0, PI/2.0))


        schedule(autoCommands)

    }
}