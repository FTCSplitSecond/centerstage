package org.firstinspires.ftc.teamcode.robot.commands

import dev.turtles.anchor.component.stock.SequentialParent
import dev.turtles.anchor.component.stock.parallel
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeState
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition

class MoveToTravel(val robot : Robot) : SequentialParent(mutableListOf(
        parallel(
                SetElbowPosition(robot.elbow, ElbowPosition.TRAVEL),
                SetTelescopePosition(robot.telescope, TelescopeState.TRAVEL),
                SetWristPosition(robot.wrist, WristPosition.TRAVEL)
        )
))