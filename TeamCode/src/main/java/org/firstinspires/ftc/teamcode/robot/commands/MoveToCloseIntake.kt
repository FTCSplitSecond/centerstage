package org.firstinspires.ftc.teamcode.robot.commands

import dev.turtles.anchor.component.stock.SequentialParent
import dev.turtles.anchor.component.stock.parallel
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeState
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition

class MoveToCloseIntake(val robot : Robot) : SequentialParent(mutableListOf(
        SetWristPosition(robot.wrist, WristPosition.CLOSE_INTAKE),
        parallel(
                SetElbowPosition(robot.elbow, ElbowPosition.CLOSE_INTAKE),
                SetTelescopePosition(robot.telescope, TelescopeState.CLOSE_INTAKE)
        )
))