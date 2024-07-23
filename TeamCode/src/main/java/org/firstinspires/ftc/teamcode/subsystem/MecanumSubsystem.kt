package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.geometry.Vector2d
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.util.clamp
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive

class MecanumSubsystem(val robot: OffseasonBot): Subsystem() {
    val hardwareMap = robot.hwMap
    var drive = CenterstageMecanumDrive(hardwareMap, robot.startPose)

    override fun init() {}

    fun set(
        xVel: Double, yVel : Double, turnVel: Double
    ) {
        val x = clamp(xVel, -1.0, 1.0)
        val y = clamp(yVel, -1.0, 1.0)
        val turn = clamp(turnVel, -1.0, 1.0)

        var input = Vector2d(x, y)
        input = input.rotateBy(Math.toDegrees(robot.driverStationOffset - drive.poseEstimate.heading))

        val centerOfRobot = Translation2d(0.0, 0.0)
        val extendedIntakeCOR = Translation2d(TelescopeConfig.TELESCOPE_EXTENDED_INTAKE_COR_X, 0.0)
        val closeIntakeCoR = Translation2d(TelescopeConfig.TELESCOPE_CLOSE_INTAKE_COR_X,0.0)
        val depositCOR = Translation2d(robot.deposit.getDepositXCenterOfRotation() + TelescopeConfig.TELESCOPE_DEPOSIT_COR_X_OFFSET, 0.0)

        val cor = when (robot.deposit.armState) {
            DepositSubsystem.State.EXTENDED_INTAKE -> extendedIntakeCOR
            DepositSubsystem.State.CLOSED_INTAKE -> closeIntakeCoR
            else -> centerOfRobot
        }

        drive.setWeightedDrivePower(Pose2d(input.x, input.y, turnVel), cor)
    }

    override fun loop() {
        drive.update()
    }

    fun poseEstimate(): Pose2d {
        return drive.poseEstimate
    }

    fun rawExternalHeading(): Double {
        return drive.rawExternalHeading
    }

    fun trajectorySequenceBuilder(currentPose: Pose2d): Any {
        return drive.trajectorySequenceBuilder(currentPose)
    }

    override fun end(reason: FinishReason) {
        OffseasonBot.lastKnownAutoPose = drive.poseEstimate
    }
}