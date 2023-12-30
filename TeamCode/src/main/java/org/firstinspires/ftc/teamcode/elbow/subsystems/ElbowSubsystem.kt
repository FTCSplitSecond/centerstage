package org.firstinspires.ftc.teamcode.elbow.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Util.IKResults
import org.firstinspires.ftc.teamcode.Util.InverseKinematics
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_HOME
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.swerve.utils.clamp


enum class ElbowPosition{
    EXTENDED_INTAKE,
    CLOSE_INTAKE,
    DEPOSIT,
    TRAVEL,
    HOME
}
class ElbowSubsystem(private val motor: DcMotorEx, private val telemetry: Telemetry, opModeType: OpModeType) : SubsystemBase() {
    constructor(robot: Robot):
            this(robot.hardwareMap.get(DcMotorEx::class.java, "elbow"), robot.telemetry, robot.opModeType)
    var isEnabled = true
    var isTelemetryEnabled = false
    init {
        register()
        if (opModeType == OpModeType.AUTONOMOUS)
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    val ELBOW_MOTOR_PPR = 751.8 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val DEGREES_PER_REVOLUTION = 360.0 * 10.0/42.0  //degrees
    val PIDTolerance = 0.5 // degrees

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
    var targetAngle : Double = 0.0
        private set
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
                targetAngle = InverseKinematics.calculateArmInverseKinematics(pixelLevel).elbowAngle
            }
            field = value
        }

    private val controller = PIDController(ElbowConfig.ELBOW_KP, ElbowConfig.ELBOW_KI, ElbowConfig.ELBOW_KD)


    fun isAtTarget() : Boolean {
        return Math.abs(targetAngle-currentAngle)<PIDTolerance
    }

    override fun periodic() {
        if (isEnabled) {
            val clampedTarget = targetAngle.clamp(
                ElbowConfig.ELBOW_MIN,
                ElbowConfig.ELBOW_MAX
            )
            // power = cos(arm_angle) * (measured_distance_from_pibot/max_distance_from_pivot) * kG
            val kG = 1.0 //tbd
            motor.power = controller.calculate(currentAngle, clampedTarget)
        } else motor.power = 0.0

        if(isTelemetryEnabled) {
            telemetry.addLine("Elbow: Telemetry Enabled")
            telemetry.addData("IsEnabled:", isEnabled)
            telemetry.addData("Target Angle Degree:", targetAngle)
            telemetry.addData("Current Angle Degree", currentAngle)
            telemetry.addData("motor.getCurrrent (mA)", motor.getCurrent(CurrentUnit.MILLIAMPS))
            telemetry.addData("motor.position", motor.currentPosition)
            telemetry.addData("motor.power", motor.power)
            telemetry.update()
        }
    }


}