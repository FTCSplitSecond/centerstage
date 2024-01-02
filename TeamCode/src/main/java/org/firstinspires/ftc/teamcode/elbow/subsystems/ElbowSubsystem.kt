package org.firstinspires.ftc.teamcode.elbow.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Util.InverseKinematics
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_HOME
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig.TELESCOPE_MAX


enum class ElbowPosition{
    EXTENDED_INTAKE,
    CLOSE_INTAKE,
    DEPOSIT,
    TRAVEL,
    HOME
}
class ElbowSubsystem(private val robot : Robot) : SubsystemBase() {

    var isEnabled = true
    var isTelemetryEnabled = false
    private val telescope = robot.telescope
    private val motor = robot.hardwareMap.get(DcMotorEx::class.java, "elbow")

    init {
        register()
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    val ELBOW_MOTOR_PPR = 751.8 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val DEGREES_PER_REVOLUTION = 360.0 * 10.0/42.0  //degrees
    val PIDTolerance = 1.0 // degrees
    var targetAngle : Double = 0.0
        private set
    val motionProfileTimer = ElapsedTime()
    var previousTarget = targetAngle
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngle, 0.0, 0.0),
        MotionState(targetAngle, 0.0, 0.0),
        { ELBOW_MAX_ANGULAR_VELOCITY },
        { ELBOW_MAX_ANGULAR_ACCELERATION },
    )
    var deltaTimer = ElapsedTime()
    private var angularX : Double = currentAngle
    private var angularV : Double = 0.0
    private var angularA : Double = 0.0
    private fun getEncoderTicksFromAngle(angle : Double) : Double {
        return angle/ DEGREES_PER_REVOLUTION * ELBOW_MOTOR_PPR // ticks
    }
    private fun getAngleFromEncoderTicks(encoderTicks : Int) : Double {
        return  encoderTicks/ ELBOW_MOTOR_PPR * DEGREES_PER_REVOLUTION + ELBOW_HOME // degrees
    }

    val currentAngle : Double
        get() {
            return getAngleFromEncoderTicks(motor.currentPosition)
        }

    var position : ElbowPosition = ElbowPosition.TRAVEL
        set(value) {
            targetAngle = when(value) {
                ElbowPosition.TRAVEL -> ElbowConfig.ELBOW_TRAVEL
                ElbowPosition.CLOSE_INTAKE -> ElbowConfig.ELBOW_CLOSE_INTAKE
                ElbowPosition.DEPOSIT -> InverseKinematics.calculateArmInverseKinematics(pixelLevel).elbowAngle
                ElbowPosition.EXTENDED_INTAKE -> ElbowConfig.ELBOW_EXTENDED_INTAKE
                ElbowPosition.HOME -> ElbowConfig.ELBOW_HOME
            }
            field = value
        }
    var pixelLevel : Int = 0
        set (value){
            if (position == ElbowPosition.DEPOSIT){
                targetAngle = InverseKinematics.calculateArmInverseKinematics(value).elbowAngle
            }
            field = value
        }

    private val controller = PIDController(ElbowConfig.ELBOW_KP, ElbowConfig.ELBOW_KI, ElbowConfig.ELBOW_KD)


    fun isAtTarget() : Boolean {
        return Math.abs(targetAngle-currentAngle)<PIDTolerance
    }

    override fun periodic() {
        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()
        val newAngularX = currentAngle
        val newAngularV = (newAngularX - angularX) / deltaT
        val newAngularA = (newAngularV - angularV) / deltaT
        angularX = newAngularX
        angularV = newAngularV
        angularA = newAngularA

        // using the current motion profile target as the starting point for the next motion profile
        // this is to ensure continuity between motion profiles (eliminates jitter where the motor as moved past current angle
        // and the motion profile will generate a first location that is backwards from the current direction of motion)
        val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
        val clampedTarget = targetAngle.clamp(
            ElbowConfig.ELBOW_MIN,
            ElbowConfig.ELBOW_MAX)
        generateMotionProfile(clampedTarget, currentMotionProfileX, angularV, angularA)

        if (isEnabled) {
            val minExtension = 13.5
            val maxTotalExtension = minExtension + TELESCOPE_MAX
            val currentTotalExtension = minExtension + telescope.currentExtensionInches
            val gravityAdjustment = Math.cos(Math.toRadians(currentAngle)) * currentTotalExtension/maxTotalExtension * ElbowConfig.KG
            motor.power = controller.calculate(currentAngle, motionProfile[motionProfileTimer.seconds()].x) + gravityAdjustment
        } else motor.power = 0.0

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow  : Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle Degree:", targetAngle)
            robot.telemetry.addData("Current Angle Degree", currentAngle)
            robot.telemetry.addData("motor.getCurrrent (mA)", motor.getCurrent(CurrentUnit.MILLIAMPS))
            robot.telemetry.addData("motor.position", motor.currentPosition)
            robot.telemetry.addData("motor.power", motor.power)
            robot.telemetry.update()
        }
    }



    private fun generateMotionProfile(target: Double, currentX: Double, currentV: Double, currentA: Double) {
        if (!(previousTarget epsilonEquals target)) {
            previousTarget = target
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(currentX, currentV, currentA),
                MotionState(target, 0.0, 0.0),
                { ELBOW_MAX_ANGULAR_VELOCITY },
                { ELBOW_MAX_ANGULAR_ACCELERATION },
            )
            motionProfileTimer.reset()
        }
    }


}