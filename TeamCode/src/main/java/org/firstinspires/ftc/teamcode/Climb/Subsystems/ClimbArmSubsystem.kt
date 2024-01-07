package org.firstinspires.ftc.teamcode.claw.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Climb.Subsystems.ClimbConfig
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class ClimbPositions {
    HELD,
    UP
}
class ClimbArmSubsystem(private val leftServo : ServoImplEx, private val rightServo : ServoImplEx, private val telemetry: Telemetry) : SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(ServoImplEx::class.java, "leftHookServo"), robot.hardwareMap.get(ServoImplEx::class.java, "rightHookServo"),
                robot.telemetry)
    private var movementStartTime = System.currentTimeMillis()
    init {
        register()
        leftServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        rightServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
    var position: ClimbPositions = ClimbPositions.HELD
        set(value) {
            leftServo.position = getServoPositionFromPulseWidth(when(value){
                ClimbPositions.HELD -> ClimbConfig.LEFT_SERVO_HELD_MICROSECONDS
                ClimbPositions.UP -> ClimbConfig.LEFT_SERVO_UP_MICROSECONDS
            }, leftServo)
            rightServo.position = getServoPositionFromPulseWidth(when(value){
                ClimbPositions.HELD -> ClimbConfig.RIGHT_SERVO_HELD_MICROSECONDS
                ClimbPositions.UP -> ClimbConfig.RIGHT_SERVO_UP_MICROSECONDS
            }, rightServo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : ServoImplEx) : Double {
        return (pulseWidth - servo.pwmRange.usPulseLower) / (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower)
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > ClawConfig.estimatedTimeToComplete
    }
    private var isTelemetryEnabled = false
    override fun periodic(){
        if(isTelemetryEnabled) {
            telemetry.update()
        }
    }

}


