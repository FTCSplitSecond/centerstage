package org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class DronePositions{
    HELD,
    LAUNCH
}

class DroneSubsystem( private val Servo : ServoImplEx, private val telemetry: Telemetry) : SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(ServoImplEx::class.java, "droneServo"),
                robot.telemetry)
    private var movementStartTime = System.currentTimeMillis()
    init {
        register()
        Servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
    var position: DronePositions = DronePositions.HELD
        set(value) {
            Servo.position = getServoPositionFromPulseWidth(when(value){
                DronePositions.HELD -> DroneConfig.HELD_MICROSECONDS
                DronePositions.LAUNCH -> DroneConfig.LAUNCH_MICROSECONDS
            }, Servo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : ServoImplEx) : Double {
        return (pulseWidth - servo.pwmRange.usPulseLower) / (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower)
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > DroneConfig.estimatedTimeToComplete
    }
    private var isTelemetryEnabled = false
    override fun periodic(){
        if(isTelemetryEnabled) {

            telemetry.update()
        }
    }

}