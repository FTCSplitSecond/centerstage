package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.apache.commons.math3.geometry.spherical.oned.ArcsSet.Split
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem

class ToggleRightClaw(private val rightClaw : RightClawSubsystem) : SplitSecondComponent() {
    override fun start() {
        rightClaw.position = when(rightClaw.position) {
            ClawPositions.OPEN -> ClawPositions.CLOSED
            ClawPositions.CLOSED -> ClawPositions.OPEN
        }

    }
    override fun isComplete() : Boolean {
        return rightClaw.movementShouldBeComplete()
    }
}