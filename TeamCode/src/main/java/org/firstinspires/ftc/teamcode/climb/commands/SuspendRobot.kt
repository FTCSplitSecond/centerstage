package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbPulleySubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.PulleyPower

class SuspendRobot(private val Suspend : ClimbPulleySubsystem) : SplitSecondComponent(){
    override fun start() {
        Suspend.power = when(Suspend.power) {
            PulleyPower .NEUTRAL -> PulleyPower.UP
            PulleyPower.UP -> PulleyPower.NEUTRAL
        }

    }
    override fun isComplete() : Boolean {
        return Suspend.movementShouldBeComplete()
    }
}