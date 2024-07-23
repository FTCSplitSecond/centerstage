package org.firstinspires.ftc.teamcode.common.component

import dev.turtles.electriceel.util.Pose
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem

class DriveMecanum(val drive: MecanumSubsystem,
                   private val xSupplier: () -> Double,
                   private val ySupplier: () -> Double,
                   private val turnSupplier: () -> Double) : SplitSecondComponent() {
    override fun loop() {
        drive.set(xSupplier(), ySupplier(), turnSupplier())
    }
}