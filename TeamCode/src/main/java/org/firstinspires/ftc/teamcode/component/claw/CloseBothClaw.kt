package org.firstinspires.ftc.teamcode.component.claw

import org.firstinspires.ftc.teamcode.common.component.SplitSecondComponent
import org.firstinspires.ftc.teamcode.common.types.ClawSide
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem

class CloseBothClaw(private val claw: ClawSubsystem): SplitSecondComponent() {
    override fun start() {
        claw.updateState(ClawSide.BOTH, ClawSubsystem.ClawState.CLOSED)
    }
}