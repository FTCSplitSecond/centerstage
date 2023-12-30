package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem

class ToggleRightClaw(private val rightClaw : RightClawSubsystem) : CommandBase(){
    init {
        addRequirements(rightClaw)
    }
    override fun initialize() {
        rightClaw.position = when(rightClaw.position) {
            ClawPositions.OPEN -> ClawPositions.CLOSED
            ClawPositions.CLOSED -> ClawPositions.OPEN
        }

    }
    override fun isFinished() : Boolean {
        return rightClaw.movementShouldBeComplete()
    }
}