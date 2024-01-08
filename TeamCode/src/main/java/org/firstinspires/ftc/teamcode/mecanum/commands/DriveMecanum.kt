package org.firstinspires.ftc.teamcode.mecanum.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.mecanum.subsystems.MecanumDriveBase

class DriveMecanum(val drive: MecanumDriveBase,
                   private val xSupplier: () -> Double,
                   private val ySupplier: () -> Double,
                   private val turnSupplier: () -> Double) : SplitSecondComponent() {
    override fun loop() {
        drive.driveFieldCentric(xSupplier(), ySupplier(), turnSupplier())
    }
}