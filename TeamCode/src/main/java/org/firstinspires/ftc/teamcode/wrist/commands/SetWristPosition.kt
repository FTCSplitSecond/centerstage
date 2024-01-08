package org.firstinspires.ftc.teamcode.wrist.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem

class SetWristPosition(private val wrist: WristSubsystem, private val position: WristPosition): SplitSecondComponent() {
    override fun start() {
        wrist.position = position
    }
    override fun isComplete(): Boolean {
        return wrist.movementShouldBeComplete()
    }
}