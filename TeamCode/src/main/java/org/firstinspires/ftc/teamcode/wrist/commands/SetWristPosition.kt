package org.firstinspires.ftc.teamcode.wrist.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPositions
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem

class SetWristPosition(private val wrist: WristSubsystem, private val position: WristPositions): CommandBase() {
    init {
        addRequirements(wrist)
    }
    override fun initialize() {
        wrist.position = position
    }
    override fun isFinished(): Boolean {
        return wrist.movementShouldBeComplete()
    }
}