package org.firstinspires.ftc.teamcode.robot.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import dev.turtles.anchor.component.stock.SequentialParent
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition

class MoveToTravel(val robot : Robot) : SequentialParent(mutableListOf(
        parallel(
                SetElbowPosition(robot.elbow, ElbowPosition.TRAVEL),
                SetTelescopePosition(robot.telescope, TelescopePosition.TRAVEL),
                SetWristPosition(robot.wrist, WristPosition.TRAVEL)
        )
))