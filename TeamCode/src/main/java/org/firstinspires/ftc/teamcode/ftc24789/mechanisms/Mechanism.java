package org.firstinspires.ftc.teamcode.ftc24789.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

abstract public class Mechanism {
    public abstract void init(HardwareMap hwMap);


    public String toString() {
        return this.getClass().getSimpleName();
    }
}
