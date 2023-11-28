package org.firstinspires.ftc.teamcode.swerve.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.AnalogInput

class SwerveModule(private val swerveDriveMotor : SwerveModuleDriveMotor,
                   private val turnServo: SwerveModuleTurnServo
) : SubsystemBase() {
   constructor(driveMotor: MotorEx, turnMotor: CRServo, angleAnalogInput: AnalogInput) :
           this(SwerveModuleDriveMotor(driveMotor), SwerveModuleTurnServo(turnMotor, angleAnalogInput))
    init {
        // register so tha the command scheduler can call periodic() where we update the drive motor
        initialize()
    }
    private var swerveModuleState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    var moduleState : SwerveModuleState
        get() = swerveModuleState
        set(value) {
            this.swerveModuleState = value
            updateMotors()
        }
    private var _isDriveMotorEnabled = true
    var isDriveMotorEnabled : Boolean
        get() = _isDriveMotorEnabled
        set(value) {
            _isDriveMotorEnabled = value
            updateMotors()
        }
    private var _isTurnServoEnabled = true
    var isTurnServoEnabled : Boolean
        get() = _isTurnServoEnabled
        set(value) {
            _isTurnServoEnabled = value
            updateMotors()
        }
    val driveMotor: SwerveModuleDriveMotor get() = swerveDriveMotor
    val turnMotor: SwerveModuleTurnServo get() = turnServo

    fun initialize() {
        turnServo.initialize()
    }
    private fun updateMotors() {
        if(_isDriveMotorEnabled)
            swerveDriveMotor.velocity = swerveModuleState.speedMetersPerSecond
        else
            swerveDriveMotor.velocity = 0.0

        if(_isTurnServoEnabled) {
            turnServo.moduleAngle = swerveModuleState.angle.radians
       }
    }

}