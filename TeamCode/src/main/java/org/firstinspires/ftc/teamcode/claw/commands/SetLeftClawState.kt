package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism

class SetLeftClawState(private val leftClaw : LeftClawSubsystem, val position : ClawPositions) : SplitSecondComponent(){
    override fun start() {
        leftClaw.position = position
    }
    override fun isComplete() : Boolean {
        return leftClaw.movementShouldBeComplete()
    }
}