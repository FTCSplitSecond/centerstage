package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.claw.commands.ToggleLeftClaw
import org.firstinspires.ftc.teamcode.claw.commands.ToggleRightClaw
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import kotlin.math.PI

//+x is left
//+y is

@Autonomous
class BlueScrimAuto: CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)
        val startPose = Pose2d(0.0, 0.0, 0.0)
        robot.driveBase.poseEstimate = startPose

        val autoCommands = SequentialCommandGroup(
            MoveToPosition(robot, Pose2d(24.0,24.0 , -PI/2)),
            SetTelescopePosition(robot.telescope, TelescopePosition.CLOSE_INTAKE),
            ToggleRightClaw(robot.rightClaw),
            SetTelescopePosition(robot.telescope, TelescopePosition.DEPOSIT),
            ToggleLeftClaw(robot.leftclaw),
            MoveToPosition(robot, Pose2d(5.0, 30.0, -PI/2))

            //MoveToPosition(robot, Pose2d(12.0,12.0, 0.0)),
            //MoveToPosition(robot, Pose2d(12.0, 12.0, -PI/2))

        )


        schedule(autoCommands)
    }
}