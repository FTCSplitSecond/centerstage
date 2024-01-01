package org.firstinspires.ftc.teamcode.robot.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition

class MoveToCloseIntake(val robot : Robot) : ConfigurableCommandBase()  {
    override fun initialize() {
        super.initialize()
    }

    override fun configure(): CommandBase {
        return SequentialCommandGroup(
//            OpenBothClaw(robot.leftclaw, robot.rightClaw),
            SetWristPosition(robot.wrist, WristPosition.CLOSE_INTAKE),
            ParallelCommandGroup(
                SetElbowPosition(robot.elbow, ElbowPosition.CLOSE_INTAKE),
                SetTelescopePosition(robot.telescope, TelescopePosition.CLOSE_INTAKE)
            )
        )
    }
}