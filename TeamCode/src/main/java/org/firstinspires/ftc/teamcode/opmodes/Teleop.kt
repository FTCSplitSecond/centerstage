package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.forever
import dev.turtles.electriceel.opmode.AnchorOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "Primary TeleOp", group = "!")
class Teleop: AnchorOpMode() {
    override fun prerun() {
        TODO("Not yet implemented")
    }

    override fun run() {
        val shapeJoystick : (Double) -> Double = {
            it.absoluteValue.pow(DriveConstants.JOYSTICK_EXPONENT) * sign(it)
        }


    }
}