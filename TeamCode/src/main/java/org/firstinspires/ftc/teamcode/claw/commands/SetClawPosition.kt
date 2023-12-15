package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawSubsystem

class SetClawPosition(private val claw : ClawSubsystem, private val position: ClawPositions) : CommandBase(){
    init {
        addRequirements(claw)
    }

    override fun execute() {
        claw.position = position
    }





}