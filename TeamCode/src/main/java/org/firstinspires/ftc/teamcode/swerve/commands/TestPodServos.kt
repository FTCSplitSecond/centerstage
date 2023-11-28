package org.firstinspires.ftc.teamcode.swerve.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class TestPodServos(private val allServos : MotorGroup,
                    private val fLA :AnalogInput, private val fRA :AnalogInput, private val bLA :AnalogInput, private val bRA :AnalogInput,
                    private val telemetry: Telemetry, private val angleProvider : () -> Rotation2d ): CommandBase() {

    private var frontLeftAngleOffset = 0.0
    private var frontRightAngleOffset = 0.0
    private var backLeftAngleOffset = 0.0
    private var backRightAngleOffset = 0.0
    private var servoWrapAngleOffset = 0.0
    private var currentWrappedServoAngle = 0.0
    private var currentModuleAngle = 0.0
    private var initialized = false
    private var initialModuleAngle = 0.0
    override fun initialize() {
        super.initialize()
    }
    override fun execute() {
        val bRServo = allServos.elementAt(3)
        val turnPIDController = PIDController(-0.01, 0.0, 0.0)

        val unwrappedServoAngle = getUnwrappedServoAngle(bRA)
        val newWrappedServoAngle =  servoWrapAngleOffset + unwrappedServoAngle

        // if the new angle is more than pi radians away from the current angle, add or subtract 2pi radians to the offset
        // this is handle the wrapping of the servo angle due to the servo gear ratio
        if (abs(newWrappedServoAngle - currentWrappedServoAngle) > 180.0)
            servoWrapAngleOffset += 360.0 * -sign(newWrappedServoAngle - currentWrappedServoAngle)
        currentWrappedServoAngle = servoWrapAngleOffset + unwrappedServoAngle

        val gearRatio = 1.0/1.0
        if(!initialized) {
            initialModuleAngle = currentWrappedServoAngle * gearRatio
            initialized = true
        }
        currentModuleAngle = (currentWrappedServoAngle * gearRatio - initialModuleAngle) % 360.0

        // run PIDController to control the turnMotor Power until the measured angle is the desired angle
        val targetModuleAngle = angleProvider().degrees
        // calculate the difference between the current angle and the target angle and normalize it to be between -180 and 180
        val angleDifference = (currentModuleAngle - targetModuleAngle + 180) % 360 - 180
        val turnMotorPower = max(-1.0, min(1.0, turnPIDController.calculate(angleDifference, 0.0)))
        bRServo.set(turnMotorPower)

        telemetry.addData("bRA.voltage", bRA.voltage )
        telemetry.addData("bRA.maxVoltage", bRA.maxVoltage )
        telemetry.addData("unwrappedServoAngle", unwrappedServoAngle )
        telemetry.addData("newWrappedServoAngle", newWrappedServoAngle )
        telemetry.addData("currentWrappedServoAngle", currentWrappedServoAngle )
        telemetry.addData("angleDifference", angleDifference )
        telemetry.addData("currentModuleAngle", currentModuleAngle )
        telemetry.addData("targetModuleAngle", targetModuleAngle )
        telemetry.addData("turnMotorPower", turnMotorPower )
        telemetry.update()
    }

    private fun getUnwrappedServoAngle(analogInput : AnalogInput): Double {
        return analogInput.voltage/analogInput.maxVoltage * 360.0
    }
}