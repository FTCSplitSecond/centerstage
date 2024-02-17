package org.firstinspires.ftc.teamcode.elbow.commands
import android.util.Log
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem

class SetElbowPosition(private val elbow : ElbowSubsystem, private val position: ElbowPosition) : SplitSecondComponent() {

    override fun start() {
        elbow.position = position
    }

    override fun isComplete(): Boolean {
        val timedOut = timer.elapsedTime > 1.0
        if(timedOut) Log.d("elbow","timeout")
        return elbow.isAtTarget() || timedOut
    }

}