package org.firstinspires.ftc.teamcode.robot.commands

import android.util.Log
import org.firstinspires.ftc.teamcode.SplitSecondComponent

class WaitFor(val condition : () -> Boolean) : SplitSecondComponent(){
    override fun isComplete(): Boolean {
        Log.d("waitfor", condition().toString())
        return condition()
    }

}