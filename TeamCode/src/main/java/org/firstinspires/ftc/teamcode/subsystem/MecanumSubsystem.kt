package org.firstinspires.ftc.teamcode.subsystem

import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.geometry.Vector2d
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.util.Pose
import dev.turtles.electriceel.util.clamp
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive

class MecanumSubsystem(val robot: OffseasonBot): Subsystem() {
    val hardwareMap = robot.hwMap
    var drive = CenterstageMecanumDrive(hardwareMap, robot.startPose)

    override fun init() {}

    fun driveField(
        pose: Pose,
        turnScalar: Double
    ) {
        val x = clamp(pose.x, -1.0, 1.0)
        val y = clamp(pose.y, -1.0, 1.0)
        val turn = clamp(turnScalar, -1.0, 1.0)

        var input = Vector2d(x, y)
        input = input.rotateBy(Math.toDegrees(robot.driverStationOffset - drive.poseEstimate.heading))

        val centerOfRobot = Translation2d(0.0, 0.0)
        val extendedIntakeCOR = Translation2d(TelescopeConfig.TELESCOPE_EXTENDED_INTAKE_COR_X, 0.0)
    }

    override fun loop() {
        drive.update()
    }

    override fun end(reason: FinishReason) {
        OffseasonBot.lastKnownAutoPose = drive.poseEstimate
    }
}