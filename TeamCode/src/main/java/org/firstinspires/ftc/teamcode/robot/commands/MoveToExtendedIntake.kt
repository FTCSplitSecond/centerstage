package org.firstinspires.ftc.teamcode.robot.commands

import dev.turtles.anchor.component.stock.SequentialParent
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeState
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition

class MoveToExtendedIntake(val robot : Robot) : SequentialParent(mutableListOf(series(
        OpenBothClaw(robot.leftClaw, robot.rightClaw),
        SetWristPosition(robot.wrist, WristPosition.EXTENDED_INTAKE),
        parallel(
                SetElbowPosition(robot.elbow, ElbowPosition.EXTENDED_INTAKE),
                SetTelescopePosition(robot.telescope, TelescopeState.EXTENDED_INTAKE)
        )
)))