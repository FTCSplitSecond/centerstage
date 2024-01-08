package org.firstinspires.ftc.teamcode.claw.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbArmSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem

class RaiseClimbArms(private val Climb : ClimbArmSubsystem) : SplitSecondComponent(){
    override fun start() {
        Climb.position = when(Climb.position) {

            ClimbPositions.HELD -> ClimbPositions.UP
            ClimbPositions.UP -> ClimbPositions.HELD
        }

    }
    override fun isComplete() : Boolean {
        return Climb.movementShouldBeComplete()
    }
}