package org.firstinspires.ftc.teamcode.telescope.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.InverseKinematics
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig.*
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.swerve.utils.clamp


enum class  TelescopePosition {
    EXTENDED_INTAKE,
    CLOSE_INTAKE,
    DEPOSIT,
    TRAVEL
}

class TelescopeSubsytem(private val robot: Robot) : SubsystemBase() {

    var isEnabled = true
    var isTelemetryEnabled = false
    private val motor = robot.hardwareMap.get(DcMotorEx::class.java, "telescope")
    init {
        register()
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    val TELESCOPE_MOTOR_PPR = 384.5 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val INCHES_PER_REVOLUTION = 30.0/25.4 * Math.PI //inches
    val PIDTolerance = 0.25 // inches

    private fun getEncoderTicksFromExtensionInches(extensionInches : Double) : Double {
        return extensionInches/ INCHES_PER_REVOLUTION * TELESCOPE_MOTOR_PPR // ticks
    }
    private fun getExtensionInchesFromEncoderTicks(encoderTicks : Int) : Double {
        val driveMotorRevolutions = encoderTicks/ TELESCOPE_MOTOR_PPR
        val elbowAngleRevolutions = (robot.elbow.currentAngle - ElbowConfig.ELBOW_HOME)/360.0
        return (driveMotorRevolutions + elbowAngleRevolutions) * INCHES_PER_REVOLUTION // inches
    }

    val  currentExtensionInches : Double
        get() {
            return getExtensionInchesFromEncoderTicks(motor.currentPosition)
        }
    var targetExtenstionInches : Double = 0.0
        private set
    var position : TelescopePosition = TelescopePosition.TRAVEL
        set(value) {
            targetExtenstionInches = when(value) {
                TelescopePosition.TRAVEL -> TELESCOPE_TRAVEL
                TelescopePosition.CLOSE_INTAKE -> TELESCOPE_CLOSE_INTAKE
                TelescopePosition.DEPOSIT -> InverseKinematics.calculateArmInverseKinematics(pixelLevel).telescopeExtension
                TelescopePosition.EXTENDED_INTAKE -> TELESCOPE_EXTENDED_INTAKE
            }
            field = value
        }
    var pixelLevel : Int = 0
        set (value){
            if (position == TelescopePosition.DEPOSIT) {
                targetExtenstionInches = InverseKinematics.calculateArmInverseKinematics(value).telescopeExtension
            }
            field = value
        }


    private val controller = PIDController(TELESCOPE_KP, TELESCOPE_KI, TELESCOPE_KD)

    fun isAtTarget() : Boolean {
        return Math.abs(targetExtenstionInches-currentExtensionInches)<PIDTolerance
    }

    override fun periodic() {
        if (isEnabled) {
            val clampedTarget = targetExtenstionInches.clamp(TELESCOPE_MIN, TELESCOPE_MAX)
            motor.power = controller.calculate(currentExtensionInches, clampedTarget)
        }

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Telescope: Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Extension Inches:", targetExtenstionInches)
            robot.telemetry.addData("Current Extension Inches", currentExtensionInches)
            robot.telemetry.addData("motor.getCurrrent (mA)", motor.getCurrent(CurrentUnit.MILLIAMPS))
            robot.telemetry.addData("motor.position", motor.currentPosition)
            robot.telemetry.addData("motor.power", motor.power)
            robot.telemetry.update()
        }
    }
}
