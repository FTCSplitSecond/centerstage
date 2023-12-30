package org.firstinspires.ftc.teamcode.mecanum.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.mecanum.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.claw.subsystems.ClawSubsystem

class SetClawPosition(private val claw : ClawSubsystem, private val position: ClawPositions) : CommandBase(){
    init {
        addRequirements(claw)
    }
    override fun initialize() {
        claw.position = position
    }
  override fun isFinished() : Boolean {
        return claw.movementShouldBeComplete()
    }
}