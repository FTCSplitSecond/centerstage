package org.firstinspires.ftc.teamcode.robot.commands

import android.util.Log
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import org.firstinspires.ftc.teamcode.Util.InverseKinematics
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class IncreasePixelLevel(val robot : Robot) : CommandBase()  {

    init {
        addRequirements(robot.telescope, robot.wrist, robot.elbow)
    }

    override fun initialize() {
        Log.i("PixelLevel","increase pre-pixelLevel = " + robot.elbow.pixelLevel)
        robot.telescope.pixelLevel = (robot.telescope.pixelLevel + 1).clamp(0, InverseKinematics.MAX_PIXEL_LEVEL)
        robot.elbow.pixelLevel = (robot.elbow.pixelLevel + 1).clamp(0, InverseKinematics.MAX_PIXEL_LEVEL)
        robot.wrist.pixelLevel = (robot.elbow.pixelLevel + 1).clamp(0, InverseKinematics.MAX_PIXEL_LEVEL)
        Log.i("PixelLevel","increase pixelLevel = " + robot.elbow.pixelLevel)
    }

    override fun isFinished(): Boolean {
//        Log.i("PixelLevel","increase isfinished robot.telescope.isAtTarget() = " + robot.telescope.isAtTarget())
//        Log.i("PixelLevel","increase isfinished robot.elbow.isAtTarget() = " + robot.elbow.isAtTarget())
//        return robot.telescope.isAtTarget() && robot.elbow.isAtTarget()
        return true
    }

}