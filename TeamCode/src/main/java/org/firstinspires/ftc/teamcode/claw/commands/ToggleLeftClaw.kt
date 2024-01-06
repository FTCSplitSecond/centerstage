package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem

class ToggleLeftClaw(private val LeftClaw : LeftClawSubsystem) : CommandBase(){
    init {
            addRequirements(LeftClaw)
    }
    override fun initialize() {
        LeftClaw.position = when(LeftClaw.position) {
            ClawPositions.OPEN -> ClawPositions.CLOSED
            ClawPositions.CLOSED -> ClawPositions.OPEN
        }

    }
    override fun isFinished() : Boolean {
        return LeftClaw.movementShouldBeComplete()
    }
}