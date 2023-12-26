package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.commands.PositionConfig.hController
import org.firstinspires.ftc.teamcode.robot.commands.PositionConfig.xController
import org.firstinspires.ftc.teamcode.robot.commands.PositionConfig.yController
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.*


class MoveToPosition(val robot: Robot, private val targetPose: Pose2d): CommandBase() {

    private val timeout = ElapsedTime()
    private val stableTime = ElapsedTime()

    override fun initialize() {
        timeout.reset()
        stableTime.reset()
    }

    override fun execute() {
        val robotPose = robot.driveBase.poseEstimate
        val powers = getPower(robotPose)
        robot.driveBase.driveFieldCentric(powers.x, powers.y, powers.heading)
    }

    override fun isFinished(): Boolean {
        val robotPose = Pose2d()
        val delta = targetPose.minus(robotPose)

        if(delta.vec().norm() > PositionConfig.ALLOWED_TRANSLATIONAL_ERROR
            || abs(delta.heading) > PositionConfig.ALLOWED_HEADING_ERROR) {
            stableTime.reset()
        }

        return timeout.milliseconds()>5000 || stableTime.milliseconds() > 250
    }

    private fun getPower(robotPose: Pose2d): Pose2d {
        val headingError = angleWrap(targetPose.heading- robotPose.heading)
        val xPower = xController.calculate(robotPose.x, targetPose.x)
        val yPower = yController.calculate(robotPose.y, targetPose.y)
        var hPower = -hController.calculate(headingError, 0.0)



        return Pose2d(xPower, yPower, hPower)
    }


    private fun angleWrap(radians: Double): Double {
        var radians = radians
        while (radians > Math.PI) {
            radians -= 2 * Math.PI
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI
        }

        return radians
    }



}