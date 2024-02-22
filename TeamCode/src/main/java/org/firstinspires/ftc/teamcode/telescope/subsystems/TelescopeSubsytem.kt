package org.firstinspires.ftc.teamcode.telescope.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig.*
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.util.adjustPowerForKStatic
import org.firstinspires.ftc.teamcode.swerve.utils.clamp




class TelescopeSubsytem(private val hardwareManager: HardwareManager, private val robot: Robot) : Subsystem() {

    var isTelemetryEnabled = false

    private val motor1 = hardwareManager.motor("telescope1")
    private val motor2 = hardwareManager.motor("telescope1")



    init {
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    val TELESCOPE_MOTOR_PPR = 384.5 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val INCHES_PER_REVOLUTION = 30.0/25.4 * Math.PI //inches
    val PIDTolerance = 1.0 // inches
    var targetExtenstionInches : Double = 0.0
    val motionProfileTimer = ElapsedTime()
    var previousTarget = targetExtenstionInches
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentExtensionInches, 0.0, 0.0),
        MotionState(targetExtenstionInches, 0.0, 0.0),
        { TELESCOPE_MAX_ACCELERATION },
        { TELESCOPE_MAX_VELOCITY },
    )
    var deltaTimer = ElapsedTime()
    private var X : Double = currentExtensionInches
    private var V : Double = 0.0
    private var A : Double = 0.0

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
            return getExtensionInchesFromEncoderTicks(motor1.encoder.getCounts())
        }


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
        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()
        val newX = currentExtensionInches
        val newV = (newX - X) / deltaT
        val newA = (newV - V) / deltaT
        X = newX
        V = newV
        A = newA
        val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
        val clampedTarget = targetExtenstionInches.clamp(TELESCOPE_MIN, TELESCOPE_MAX)
        generateMotionProfile(clampedTarget, currentMotionProfileX, V, A)
        val pidPower = controller.calculate(currentExtensionInches, motionProfile[motionProfileTimer.seconds()].x).adjustPowerForKStatic(TELESCOPE_KS)

        motor1 power pidPower
        motor2 power pidPower

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Telescope: Telemetry Enabled")
            robot.telemetry.addData("Target Extension Inches:", targetExtenstionInches)
            robot.telemetry.addData("Current Extension Inches", currentExtensionInches)
            robot.telemetry.addData("Extension Error Inches", targetExtenstionInches - currentExtensionInches)
            robot.telemetry.addData("Is At Target", this.isAtTarget())
            robot.telemetry.addData("motor.getCurrrent (mA)", motor1.getCurrent() * 1000.0)
            robot.telemetry.addData("motor.position", motor1.encoder.getCounts())
        }
    }

    override fun end(reason: FinishReason) {

    }

    private fun generateMotionProfile(target: Double, currentX: Double, currentV: Double, currentA: Double) {
        if (!(previousTarget epsilonEquals target)) {
            previousTarget = target
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(currentX, currentV, currentA),
                MotionState(target, 0.0, 0.0),
                { TelescopeConfig.TELESCOPE_MAX_VELOCITY},
                { TelescopeConfig.TELESCOPE_MAX_ACCELERATION },
            )
            motionProfileTimer.reset()
        }
    }
}
