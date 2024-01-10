package org.firstinspires.ftc.teamcode.claw.commands

import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem

class ToggleLeftClaw(private val leftClaw : LeftClawSubsystem) : SplitSecondComponent(){
    override fun start() {
        leftClaw.position = when(leftClaw.position) {
            ClawPositions.OPEN -> ClawPositions.CLOSED
            ClawPositions.CLOSED -> ClawPositions.OPEN
        }

    }
    override fun isComplete() : Boolean {
        return leftClaw.movementShouldBeComplete()
    }
}