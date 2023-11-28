package org.firstinspires.ftc.teamcode.swerve.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.qualcomm.robotcore.hardware.AnalogInput

class SwerveDriveBase(frontLeftSwerveModule : SwerveModule,
                      frontRightSwerveModule : SwerveModule,
                      backLeftSwerveModule : SwerveModule,
                      backRightSwerveModule: SwerveModule,
                      private val gyroAngleProvider : () -> Rotation2d) : SubsystemBase() {
    constructor(frontLeftDrive : MotorEx, frontRightDrive : MotorEx, backLeftDrive : MotorEx, backRightDrive : MotorEx,
                frontLeftServo : CRServo, frontRightServo : CRServo, backLeftServo : CRServo, backRightServo : CRServo,
                frontLeftServoAngle : AnalogInput, frontRightServoAngle : AnalogInput,
                backLeftServoAngle : AnalogInput, backRightServoAngle : AnalogInput,
                gyroAngleProvider : () -> Rotation2d)
            : this(
        SwerveModule(frontLeftDrive, frontLeftServo, frontLeftServoAngle),
                   SwerveModule(frontRightDrive, frontRightServo, frontRightServoAngle),
                   SwerveModule(backLeftDrive, backLeftServo, backLeftServoAngle),
                   SwerveModule(backRightDrive, backRightServo, backRightServoAngle),
                   gyroAngleProvider)

    // The locations of the swerve modules relative to the robot center (measurements in meters, taken from OnShape)
    // TODO: these might need to be moved to SwerveDriveConfiguration.kt
    private val mainDimension = 130.9 / 1000.0
    private val frontLeftLocation = Translation2d(mainDimension, mainDimension)
    private val frontRightLocation = Translation2d(mainDimension, -mainDimension)
    private val backLeftLocation = Translation2d(-mainDimension, mainDimension)
    private val backRightLocation = Translation2d(-mainDimension, -mainDimension)

    // The kinematics object that converts between chassis speeds and module states
    private val kinematics = SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation)
    private val swerveModules = arrayOf(frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule)
    private val swerveOdometry = com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry(kinematics, gyroAngleProvider())
    private var moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 0.0))

    init {
        register()

        // invert the left drive motors
        frontLeftSwerveModule.driveMotor.inverted = true
        backLeftSwerveModule.driveMotor.inverted = true
    }

    override fun periodic() {
        // get the current time in seconds
        val currentTime = System.currentTimeMillis() / 1000.0
        swerveOdometry.updateWithTime(currentTime, gyroAngleProvider(), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3])
    }
    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Convert the desired chassis speeds into module states
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        swerveModules.forEachIndexed { index, swerveModule -> swerveModule.moduleState = moduleStates[index] }
    }

    fun initialize(initialPose : Pose2d) {
        // this will assume that all modules are point forward (+x) and will assign this the 0 degree position
        // to be called in the OpMode initialize() method
        swerveModules.forEach { it.initialize() }
        swerveOdometry.resetPosition(initialPose, gyroAngleProvider())
     }
}