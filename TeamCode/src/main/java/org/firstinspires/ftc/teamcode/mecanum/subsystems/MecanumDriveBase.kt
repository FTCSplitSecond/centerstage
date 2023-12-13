package org.firstinspires.ftc.teamcode.mecanum.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.util.MathUtils
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class MecanumDriveBase(hardwareMap: HardwareMap, val telemetry: Telemetry) : CenterstageMecanumDrive(hardwareMap), Subsystem {
    constructor(robot: Robot) : this(robot.hardwareMap, robot.telemetry)
    init {
        this.register()
    }
    fun driveFieldCentric(forwardSpeed: Double, strafeSpeed: Double, turnSpeed: Double) {
        val forwardSpeed = MathUtils.clamp(forwardSpeed, -1.0, 1.0)
        val strafeSpeed = MathUtils.clamp(strafeSpeed, -1.0, 1.0)
        val turnSpeed = MathUtils.clamp(turnSpeed, -1.0, 1.0)

        var input = com.arcrobotics.ftclib.geometry.Vector2d(forwardSpeed, strafeSpeed)
        input = input.rotateBy(Math.toDegrees(this.rawExternalHeading))
        setWeightedDrivePower(Pose2d(input.x, -input.y, -turnSpeed))
    }
    override fun periodic() {
        update()
    }
}