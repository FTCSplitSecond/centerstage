package org.firstinspires.ftc.teamcode.component.elbow

import org.firstinspires.ftc.teamcode.common.component.SplitSecondComponent
import org.firstinspires.ftc.teamcode.subsystem.elbow.ElbowPositions
import org.firstinspires.ftc.teamcode.subsystem.elbow.ElbowSubsystem

class SetElbowComponent(private val elbow: ElbowSubsystem, val position: ElbowPositions): SplitSecondComponent() {
    override fun start() {
        elbow.position = position
    }

    override fun isComplete(): Boolean {
        val timedOut = timer.elapsedTime > 1.0
        if (timedOut) println("timed out")
        return elbow.inThresh || timedOut
    }
}