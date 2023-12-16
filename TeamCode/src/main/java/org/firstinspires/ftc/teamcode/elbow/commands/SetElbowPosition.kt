package org.firstinspires.ftc.teamcode.elbow.commands
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem

class SetElbowPosition(private val elbow : ElbowSubsystem, private val position: ElbowPosition) : CommandBase() {

    init {
        addRequirements(elbow)
    }

    override fun initialize() {
        elbow.position = position
    }

    override fun isFinished(): Boolean {
        return elbow.isAtTarget()
    }

}