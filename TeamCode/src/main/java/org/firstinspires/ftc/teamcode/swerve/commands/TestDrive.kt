package org.firstinspires.ftc.teamcode.swerve.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.swerve.subsystems.SwerveDriveBase
import java.util.function.DoubleSupplier

class TestDrive(private val leapfrog : SwerveDriveBase, private val forward : DoubleSupplier) : CommandBase() {
    init {
        addRequirements(leapfrog);
    }
    override fun initialize() {
          leapfrog.initialize(Pose2d(0.0, 0.0, Rotation2d(0.0)))
    }
    override fun execute() {
        // just drive forward/back +/- X-direction
        val chassisSpeeds = ChassisSpeeds(forward.asDouble, 0.0, 0.0)
        leapfrog.drive(chassisSpeeds)
    }
}