package org.firstinspires.ftc.teamcode.claw.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.climb.Subsystems.ClimbConfig
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class PulleyPower {
    NEUTRAL,
    UP
}
class ClimbPulleySubsystem(private val leftServo : CRServoImplEx, private val rightServo : CRServoImplEx, private val telemetry: Telemetry) : SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(CRServoImplEx::class.java, "leftClimbServo"), robot.hardwareMap.get(CRServoImplEx::class.java, "rightClimbServo"),
                robot.telemetry)
    private var movementStartTime = System.currentTimeMillis()
    init {
        register()
        leftServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        rightServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
    var power: PulleyPower = PulleyPower.NEUTRAL
        set(value) {
            leftServo.power = (when(value){
                PulleyPower.NEUTRAL -> ClimbConfig.NEUTRAL_POS
                PulleyPower.UP -> ClimbConfig.SUSPEND_ROBOT
            })
            rightServo.power = (when(value){
                PulleyPower.NEUTRAL -> ClimbConfig.NEUTRAL_POS
                PulleyPower.UP -> ClimbConfig.SUSPEND_ROBOT
            })
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


