package org.firstinspires.ftc.teamcode.swerve.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.swerve.commands.SwerveDrive
import org.firstinspires.ftc.teamcode.swerve.subsystems.SwerveDriveBase
import org.firstinspires.ftc.teamcode.swerve.subsystems.SwerveDriveConfiguration


@TeleOp(name = "Field Relative Drive Test", group = "none")
class FieldRelativeDriveOpMode() : CommandOpMode()  {

    override fun initialize() {

        // let's get the drive motors
        val frontLeftDrive = MotorEx(hardwareMap, "frontLeftDrive", Motor.GoBILDA.BARE)
        val frontRRightDrive = MotorEx(hardwareMap, "frontRightDrive", Motor.GoBILDA.BARE)
        val backLeftDrive = MotorEx(hardwareMap, "backLeftDrive", Motor.GoBILDA.BARE)
        val backRightDrive = MotorEx(hardwareMap, "backRightDrive", Motor.GoBILDA.BARE)

        // now the servos
        val frontLeftCRServo = CRServo(hardwareMap, "frontLeftServo")
        val frontRightCRServo = CRServo(hardwareMap, "frontRightServo")
        val backLeftCRServo = CRServo(hardwareMap, "backLeftServo")
        val backRightCRServo = CRServo(hardwareMap, "backRightServo")

        // and the analog inputs
        val frontLeftAnalogInput = hardwareMap.get(AnalogInput::class.java, "frontLeftServoAngle")
        val frontRightAnalogInput = hardwareMap.get(AnalogInput::class.java, "frontRightServoAngle")
        val backLeftAnalogInput = hardwareMap.get(AnalogInput::class.java,"backLeftServoAngle")
        val backRightAnalogInput = hardwareMap.get(AnalogInput::class.java,"backRightServoAngle")
        val imu = hardwareMap.get("imu") as IMU
        val imuParameters = IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                                                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT))
        imu.initialize(imuParameters)
        imu.resetYaw()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // and now the swerve drive base
        var gyroAngleProvider = {
            Rotation2d(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))
        }
        val leapfrogDrive = SwerveDriveBase(frontLeftDrive, frontRRightDrive, backLeftDrive, backRightDrive,
            frontLeftCRServo, frontRightCRServo, backLeftCRServo, backRightCRServo,
            frontLeftAnalogInput, frontRightAnalogInput, backLeftAnalogInput, backRightAnalogInput, gyroAngleProvider)

        // Now create the command
        val driverOp = GamepadEx(gamepad1);
        val iniitalPose = Pose2d(0.0, 0.0, gyroAngleProvider())
        val command = SwerveDrive(leapfrogDrive, iniitalPose) {
            val vxMetersPerSecond = driverOp.leftX * SwerveDriveConfiguration.maxWheelSpeed
            val vyMetersPerSecond = driverOp.leftY * SwerveDriveConfiguration.maxWheelSpeed
            val omegaRadiansPerSecond = driverOp.rightY * SwerveDriveConfiguration.maxAngularSpeed
            val robotYawAngle = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            val robotRotation2d = Rotation2d(robotYawAngle)

            telemetry.addData("vx", vxMetersPerSecond)
            telemetry.addData("vy", vyMetersPerSecond)
            telemetry.addData("omega", omegaRadiansPerSecond)
            telemetry.addData("robotRotation2d.degrees", robotRotation2d.degrees)
            telemetry.update()

            return@SwerveDrive ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotRotation2d)
        }

        schedule(command);
    }
}