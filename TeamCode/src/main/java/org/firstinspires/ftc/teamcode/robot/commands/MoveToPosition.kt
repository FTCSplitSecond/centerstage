package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.abs


class MoveToPosition(val robot: Robot, private val targetPose: Pose2d): CommandBase() {

    private val timeout = ElapsedTime()
    private val stableTime = ElapsedTime()
    val xController = PIDFController(PositionConfig.xP, 0.0, PositionConfig.xD, 0.0)
    val yController = PIDFController(PositionConfig.yP, 0.0, PositionConfig.yD, 0.0)
    val hController = PIDFController(PositionConfig.hP, 0.0, PositionConfig.hD, 0.0)

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
        val headingError = AngleUnit.normalizeRadians(targetPose.heading- robotPose.heading)
        val xPower = xController.calculate(robotPose.x, targetPose.x)
        val yPower = yController.calculate(robotPose.y, targetPose.y)
        val hPower = -hController.calculate(headingError, 0.0)
        return Pose2d(xPower, yPower, hPower)
    }



}