package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem

class CloseBothClaw(private val leftClaw : LeftClawSubsystem, private val rightClaw : RightClawSubsystem) : CommandBase(){
    init {
        addRequirements(leftClaw, rightClaw)
    }
    override fun initialize() {
        leftClaw.position = ClawPositions.CLOSED
        rightClaw.position = ClawPositions.CLOSED
    }
    override fun isFinished() : Boolean {
        return leftClaw.movementShouldBeComplete() && rightClaw.movementShouldBeComplete()
    }
}