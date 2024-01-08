package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem

class CloseBothClaw(private val leftClaw : LeftClawSubsystem, private val rightClaw : RightClawSubsystem) : SplitSecondComponent(){
    override fun start() {
        leftClaw.position = ClawPositions.CLOSED
        rightClaw.position = ClawPositions.CLOSED
    }
    override fun isComplete() : Boolean {
        return leftClaw.movementShouldBeComplete() && rightClaw.movementShouldBeComplete()
    }
}