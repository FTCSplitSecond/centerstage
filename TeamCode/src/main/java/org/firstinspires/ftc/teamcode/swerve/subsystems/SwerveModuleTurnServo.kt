package org.firstinspires.ftc.teamcode.swerve.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import kotlin.math.abs
import kotlin.math.sign

class SwerveModuleTurnServo(private val turnMotor: CRServo, private val angleAnalogInput: AnalogInput) : SubsystemBase() {
    private val turnPIDController = PIDController(3.0, 0.0, 0.0)

    private var servoWrapAngleOffset = 0.0
    private var currentWrappedServoAngle = 0.0

    private var currentModuleAngle = 0.0
    private var targetModuleAngle = 0.0

    private var initialized = false
    private var initialModuleAngle = 0.0
    private val gearRatio = 1.0/1.0
    val turnPIDAngleTolerance = 2 * Math.PI / 360.0 / 2 // 1/2 degree tolerance for the PID controller
    init {
        turnPIDController.setTolerance(turnPIDAngleTolerance)
        // register this subsystem with the command scheduler so that the periodic method is called
        register()
    }
    var moduleAngle : Double
        get() = currentModuleAngle
        set(value) {
            targetModuleAngle = value
        }
    private var _atSetPoint = false
    val atSetPoint : Boolean
        get() { return _atSetPoint }
    fun initialize() {
        servoWrapAngleOffset = 0.0
        updateCurrentModuleAngle()
        initialModuleAngle = currentModuleAngle
        currentModuleAngle = 0.0
        initialized = true
    }
    override fun periodic() {
           if (initialized) {

               // update the current module angle
               updateCurrentModuleAngle()

               // calculate the difference between the current angle and the target angle and normalize it to be between -180 and 180
               val angleDifference = (currentModuleAngle - targetModuleAngle + Math.PI) % (2 * Math.PI) - Math.PI

               // now run the PID controller to turn the module to the target angle
               val pidOutput = turnPIDController.calculate(angleDifference, 0.0)
               val turnMotorPower = pidOutput.clamp(-1.0,1.0)

               _atSetPoint = turnPIDController.atSetPoint()
               if(!_atSetPoint)
                   turnMotor.set(turnMotorPower)
               else
                   turnMotor.set(0.0)
           }
    }
    private fun updateCurrentModuleAngle() {
        val unwrappedServoAngle = getUnwrappedServoAngle()
        val newWrappedServoAngle =  servoWrapAngleOffset + unwrappedServoAngle

        // if the new angle is more than pi radians away from the current angle, add or subtract 2pi radians to the offset
        // this is handle the wrapping of the servo angle due to the servo gear ratio
        if (abs(newWrappedServoAngle - currentWrappedServoAngle) > Math.PI)
            servoWrapAngleOffset += 2 * Math.PI * -sign(newWrappedServoAngle - currentWrappedServoAngle)

        currentWrappedServoAngle = servoWrapAngleOffset + unwrappedServoAngle
        currentModuleAngle = (currentWrappedServoAngle * gearRatio - initialModuleAngle) % (2 * Math.PI)
    }
    private fun getUnwrappedServoAngle(): Double {
        return angleAnalogInput.voltage/angleAnalogInput.maxVoltage * 2 * Math.PI
    }
}
