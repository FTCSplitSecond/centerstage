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
import kotlin.math.abs


class TelescopeSubsystem(hardwareManager: HardwareManager, private val robot: Robot) : Subsystem() {

    var isTelemetryEnabled = false
    var isEnabled = true
    var isHoming = false
    private var hasHomingPowerBeenSet = false

    private val motor1 = hardwareManager.motor("telescope1")
    private val motor2 = hardwareManager.motor("telescope2")

    init {
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

//        motor1.reverse(true)
//        motor2.reverse(true)
    }

    private val TELESCOPE_MOTOR_PPR = 145.1 * (18.0/19.0) // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    private val INCHES_PER_REVOLUTION = 30.0/25.4 * Math.PI //inches
    private val pidTolerance = 1.0 // inches
    private var targetExtensionInches : Double = 0.0
    private val motionProfileTimer = ElapsedTime()
    private var previousTarget = targetExtensionInches
    private var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(0.0, 0.0, 0.0),
        MotionState(0.0, 0.0, 0.0),
        { TELESCOPE_MAX_ACCELERATION },
        { TELESCOPE_MAX_VELOCITY },
    )
    private var deltaTimer = ElapsedTime()
    private var X : Double = 0.0
    private var V : Double = 0.0
    private var A : Double = 0.0

    private fun getEncoderTicksFromExtensionInches(extensionInches : Double) : Double {
        return extensionInches/ INCHES_PER_REVOLUTION * TELESCOPE_MOTOR_PPR // ticks
    }
    private fun getExtensionInchesFromEncoderTicks(encoderTicks : Double) : Double {
        val driveMotorRevolutions = /*-*/encoderTicks / TELESCOPE_MOTOR_PPR //NEGATIVE BECAUSE OF REVERSED MOTOR DIRECTION
        val elbowAngleRevolutions: Double = (robot.elbow.currentAngle - ElbowConfig.ELBOW_HOME) / 360.0
        return (driveMotorRevolutions + elbowAngleRevolutions) * INCHES_PER_REVOLUTION // inches
    }

    var currentExtensionInches : Double = 0.0
        private set

    var position : TelescopePosition = TelescopePosition.Travel
        set(value) {
            targetExtensionInches = value.extension
            field = value
        }


    private val controller = PIDController(TELESCOPE_KP, TELESCOPE_KI, TELESCOPE_KD)

    fun isAtTarget() : Boolean {
        return abs(targetExtensionInches-currentExtensionInches) < pidTolerance
    }

    override fun init() {

    }

    override fun loop() {
        currentExtensionInches = getExtensionInchesFromEncoderTicks(motor1.encoder.getCounts())
        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()
        val newX = currentExtensionInches
        val newV = (newX - X) / deltaT
        val newA = (newV - V) / deltaT
        X = newX
        V = newV
        A = newA
        val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
        val clampedTarget = targetExtensionInches.clamp(TELESCOPE_MIN, TELESCOPE_MAX)
        generateMotionProfile(clampedTarget, currentMotionProfileX, V, A)
        val pidPower = controller.calculate(currentExtensionInches, motionProfile[motionProfileTimer.seconds()].x).adjustPowerForKStatic(TELESCOPE_KS)

        if(isEnabled) {
            if(isHoming) {
                // if homing, we firs set the motor powers to a small/slow negative power to retract the telescope
                // then we check if the velocity is below a certain threshold, if it is, them the telescope has hit its end stop
                // and we set the motor powers to 0.0, reset the encoder like on startup and set isHoming to false (resume pid control on next loop)

                if(!hasHomingPowerBeenSet) {
                    motor1 power HOMING_POWER
                    motor2 power HOMING_POWER
                    hasHomingPowerBeenSet = true
                } else if(abs(V) < HOMING_VELOCITY_THRESHOLD) {
                    motor1 power 0.0
                    motor2 power 0.0
                    // we are at the end stop, reset the encoders, re-initialize the current/target
                    // and end homing sequence
                    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
                    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
                    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    currentExtensionInches = 0.0
                    position = TelescopePosition.Travel
                    isHoming = false
                    hasHomingPowerBeenSet = false
                } else {
                    motor1 power HOMING_POWER
                    motor2 power HOMING_POWER
                }
            } else {
                motor1 power pidPower
                motor2 power pidPower
            }
        } else {
            motor1 power 0.0
            motor2 power 0.0
        }

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Telescope: Telemetry Enabled")
            robot.telemetry.addData("Target Extension Inches:", targetExtensionInches)
            robot.telemetry.addData("Current Extension Inches", currentExtensionInches)
            robot.telemetry.addData("Extension Error Inches", targetExtensionInches - currentExtensionInches)
            robot.telemetry.addData("Is At Target", this.isAtTarget())
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
                { TELESCOPE_MAX_VELOCITY },
                { TELESCOPE_MAX_ACCELERATION },
            )
            motionProfileTimer.reset()
        }
    }
}
