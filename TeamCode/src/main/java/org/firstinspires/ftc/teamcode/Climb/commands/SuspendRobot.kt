package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.Climb.Subsystems.ClimbConfig
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbArmSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbPulleySubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.PulleyPower

class SuspendRobot(private val Suspend : ClimbPulleySubsystem) : CommandBase(){
    init {
        addRequirements(Suspend)
    }
    override fun initialize() {
        Suspend.power = when(Suspend.power) {
            PulleyPower .NEUTRAL -> PulleyPower.UP
            PulleyPower.UP -> PulleyPower.NEUTRAL
        }

    }
    override fun isFinished() : Boolean {
        return Suspend.movementShouldBeComplete()
    }
}