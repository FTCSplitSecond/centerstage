package org.firstinspires.ftc.teamcode.robot.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.mecanum.claw.commands.SetClawPosition
import org.firstinspires.ftc.teamcode.mecanum.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition

class MoveToExtendedIntake(val robot : Robot) : ConfigurableCommandBase()  {
    override fun initialize() {
        super.initialize()
    }

    override fun configure(): CommandBase {
        return SequentialCommandGroup(
            SetClawPosition(robot.claw, ClawPositions.OPEN),
            SetWristPosition(robot.wrist, WristPosition.EXTENDED_INTAKE),
            ParallelCommandGroup(
                SetElbowPosition(robot.elbow, ElbowPosition.EXTENDED_INTAKE),
                SetTelescopePosition(robot.telescope, TelescopePosition.EXTENDED_INTAKE)
            )
        )
    }
}