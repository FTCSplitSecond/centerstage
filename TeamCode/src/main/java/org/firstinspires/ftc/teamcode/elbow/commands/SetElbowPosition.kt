package org.firstinspires.ftc.teamcode.elbow.commands
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem

class SetElbowPosition(private val elbow : ElbowSubsystem, private val position: ElbowPosition) : SplitSecondComponent() {

    override fun start() {
        elbow.position = position
    }

    override fun isComplete(): Boolean {
        return elbow.isAtTarget()
    }

}