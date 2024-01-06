package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.robot.commands.IncreasePixelLevel
import org.firstinspires.ftc.teamcode.robot.commands.MoveToDeposit
import org.firstinspires.ftc.teamcode.robot.commands.MoveToExtendedIntake
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPlace
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPosition
import org.firstinspires.ftc.teamcode.robot.commands.MoveToTravel
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.PI

@Autonomous
class BluePlaceAuto: CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)

        val startPose = Pose2d(0.0, 0.0, 0.0)
        robot.driveBase.poseEstimate = startPose



        val autoCommands = SequentialCommandGroup(
            CloseBothClaw(robot.leftclaw, robot.rightClaw),
            MoveToTravel(robot),
            MoveToPosition(robot, Pose2d(26.0, 24.0, -PI/2)),
            MoveToDeposit(robot),
            MoveToPosition(robot, Pose2d(26.0, 28.0, -PI/2)),
            MoveToPlace(robot),
            MoveToPosition(robot, Pose2d(6.0,30.0 , 0.0))


        )
        schedule(autoCommands)
    }
}