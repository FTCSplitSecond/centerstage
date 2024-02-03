package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism

class SetRightClawState(private val rightClaw : RightClawSubsystem, val position : ClawPositions) : SplitSecondComponent(){
    override fun start() {
        rightClaw.position = position
    }
    override fun isComplete() : Boolean {
        return rightClaw.movementShouldBeComplete()
    }
}