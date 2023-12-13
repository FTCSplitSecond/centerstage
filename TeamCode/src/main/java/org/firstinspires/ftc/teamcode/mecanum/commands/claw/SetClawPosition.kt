package org.firstinspires.ftc.teamcode.mecanum.commands.claw

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.mecanum.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.subsystems.ClawSubsystem

class SetClawPosition(private val claw : ClawSubsystem, private val position: ClawPositions) : CommandBase(){

}