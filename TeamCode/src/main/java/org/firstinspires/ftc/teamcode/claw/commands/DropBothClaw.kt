package org.firstinspires.ftc.teamcode.claw.commands

import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem

class DropBothClaw(private val leftClaw : LeftClawSubsystem, private val rightClaw : RightClawSubsystem) : SplitSecondComponent() {
    override fun start() {
        leftClaw.position = ClawPositions.DROP
        rightClaw.position = ClawPositions.DROP
    }
    override fun isComplete() : Boolean {
        return leftClaw.movementShouldBeComplete() && rightClaw.movementShouldBeComplete()
    }
}