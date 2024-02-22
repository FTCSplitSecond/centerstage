package org.firstinspires.ftc.teamcode.telescope.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig.*
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.swerve.utils.clamp




class TelescopeSubsytem(private val hardwareManager: HardwareManager, private val robot: Robot) : Subsystem() {

    var isTelemetryEnabled = false

    private val motor = hardwareManager.motor("telescope")

    init {
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    val TELESCOPE_MOTOR_PPR = 384.5 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val INCHES_PER_REVOLUTION = 30.0/25.4 * Math.PI //inches
    val PIDTolerance = 1.0 // inches

    private fun getEncoderTicksFromExtensionInches(extensionInches : Double) : Double {
        return extensionInches/ INCHES_PER_REVOLUTION * TELESCOPE_MOTOR_PPR // ticks
    }
    private fun getExtensionInchesFromEncoderTicks(encoderTicks : Double) : Double {
        val driveMotorRevolutions = encoderTicks / TELESCOPE_MOTOR_PPR
        val elbowAngleRevolutions: Double = (robot.elbow.currentAngle - ElbowConfig.ELBOW_HOME) / 360.0
        return (driveMotorRevolutions + elbowAngleRevolutions) * INCHES_PER_REVOLUTION // inches
    }

    val currentExtensionInches : Double
        get() {
            return getExtensionInchesFromEncoderTicks(motor.encoder.getCounts())
        }
    var targetExtenstionInches : Double = 0.0

    /**
     * Angle when the current armState is [TelescopePosition.ADJUST]
     */
    var depositDistance = 0.0

    var position : TelescopePosition = TelescopePosition.Travel
        set(value) {
            targetExtenstionInches = value.extension
            field = value
        }


    private val controller = PIDController(TELESCOPE_KP, TELESCOPE_KI, TELESCOPE_KD)

    fun isAtTarget() : Boolean {
        return Math.abs(targetExtenstionInches-currentExtensionInches)<PIDTolerance
    }

    override fun init() {

    }

    override fun loop() {
        val clampedTarget = targetExtenstionInches.clamp(TELESCOPE_MIN, TELESCOPE_MAX)
        motor power controller.calculate(currentExtensionInches, clampedTarget)

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Telescope: Telemetry Enabled")
            robot.telemetry.addData("Target Extension Inches:", targetExtenstionInches)
            robot.telemetry.addData("Current Extension Inches", currentExtensionInches)
            robot.telemetry.addData("Extension Error Inches", targetExtenstionInches - currentExtensionInches)
            robot.telemetry.addData("Is At Target", this.isAtTarget())
            robot.telemetry.addData("motor.getCurrrent (mA)", motor.getCurrent() * 1000.0)
            robot.telemetry.addData("motor.position", motor.encoder.getCounts())
        }
    }

    override fun end(reason: FinishReason) {

    }
}
