package org.firstinspires.ftc.teamcode.elbow.commands
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem

class SetElbowPosition(private val elbow : ElbowSubsystem, private val position: ElbowPosition) : SplitSecondComponent() {

    override fun start() {
        elbow.position = position
    }

    override fun isComplete(): Boolean {
        return elbow.isAtTarget() || this.timer.elapsedTime > 1.0
    }

}