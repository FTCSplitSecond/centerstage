package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbArmSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem

class RaiseClimbArms(private val Climb : ClimbArmSubsystem) : CommandBase(){
    init {
        addRequirements(Climb)
    }
    override fun initialize() {
        Climb.position = when(Climb.position) {

            ClimbPositions.HELD -> ClimbPositions.UP
            ClimbPositions.UP -> ClimbPositions.HELD
        }

    }
    override fun isFinished() : Boolean {
        return Climb.movementShouldBeComplete()
    }
}