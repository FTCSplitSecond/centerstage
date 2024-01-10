package org.firstinspires.ftc.teamcode.mecanum.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.util.MathUtils
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class MecanumDriveBase(hardwareMap: HardwareMap, val telemetry: Telemetry) : Subsystem() {
    constructor(robot: Robot) : this(robot.hardwareMap, robot.telemetry)

    private val dt  = CenterstageMecanumDrive(hardwareMap)

    fun driveFieldCentric(xVel: Double, yVel : Double, turnVel: Double) {
        val xVelocity = MathUtils.clamp(xVel, -1.0, 1.0)
        val yVelocity = MathUtils.clamp(yVel, -1.0, 1.0)
        val turnVelocity = MathUtils.clamp(turnVel, -1.0, 1.0)

        var input = com.arcrobotics.ftclib.geometry.Vector2d(xVelocity, yVelocity)
        input = input.rotateBy(Math.toDegrees(-dt.rawExternalHeading))
        dt.setWeightedDrivePower(Pose2d(input.x, input.y, turnVelocity))
    }

    override fun end(reason: FinishReason) {

    }

    override fun init() {

    }

    override fun loop() {
        dt.update()
    }

    fun dt() = dt

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