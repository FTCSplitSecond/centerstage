package org.firstinspires.ftc.teamcode.mecanum.subsystems

import androidx.core.math.MathUtils
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.geometry.Translation2d
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
//import org.apache.commons.math3.util.MathUtils
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism

class MecanumDriveBase(val robot : Robot) : Subsystem() {
    val hardwareMap = robot.hardwareMap
    val telemetry = robot.telemetry
    var dt = CenterstageMecanumDrive(hardwareMap, robot.startPose)

    fun driveFieldCentric(xVel: Double, yVel : Double, turnVel: Double) {
        val xVelocity = MathUtils.clamp(xVel, -1.0, 1.0)
        val yVelocity = MathUtils.clamp(yVel, -1.0, 1.0)
        val turnVelocity = MathUtils.clamp(turnVel, -1.0, 1.0)

        var input = com.arcrobotics.ftclib.geometry.Vector2d(xVelocity, yVelocity)
        input = input.rotateBy(Math.toDegrees(robot.awayFromDriverStationHeading - dt.poseEstimate.heading))

        val depressionAngle = 5.0 //deg
        val baseTelescopeLength = 8.0 //in
        val telescopeBaseToCenter = 4.0 //in
        val centerOfRotation = when {
            robot.scoringMechanism.armState == ScoringMechanism.State.EXTENDED_INTAKE -> {
                Translation2d(
                    Math.cos(Math.toRadians(depressionAngle)) * (robot.telescope.currentExtensionInches + baseTelescopeLength) - telescopeBaseToCenter,
                    0.0
                )
            }
            else -> Translation2d(0.0, 0.0)
        }

        dt.setDrivePower(Pose2d(input.x, input.y, turnVelocity), centerOfRotation)
    }

    override fun end(reason: FinishReason) {
        Robot.lastKnownAutoPose = dt.poseEstimate
    }

    override fun init() {

    }

    override fun loop() {
        dt.update()
    }


    fun poseEstimate(): Pose2d {
       return dt.poseEstimate
    }

    fun rawExternalHeading(): Double {
        return dt.rawExternalHeading
    }

    fun trajectorySequenceBuilder(currentPose: Pose2d): Any {
        return dt.trajectorySequenceBuilder(currentPose)
    }
}