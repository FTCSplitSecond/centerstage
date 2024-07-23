package org.firstinspires.ftc.teamcode.component.wrist

import org.firstinspires.ftc.teamcode.common.component.SplitSecondComponent
import org.firstinspires.ftc.teamcode.subsystem.wrist.WristPositions
import org.firstinspires.ftc.teamcode.subsystem.wrist.WristSubsystem

class SetWristPosition(private val wrist: WristSubsystem, val position: WristPositions): SplitSecondComponent() {
    override fun start() {
        wrist.position = position
    }

    override fun isComplete(): Boolean = wrist.shouldBeComplete
}