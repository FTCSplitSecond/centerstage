package org.firstinspires.ftc.teamcode.claw.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConfig {

    public static double LEFT_SERVO_OPEN_MICROSECONDS = 1100;
    public static double LEFT_SERVO_CLOSED_TELEOP_MICROSECONDS = 2200; // VALUE FOR AUTO ONLY: decrease to 1900 if the servo is under intensive use
     public static double LEFT_SERVO_DROP_MICROSECONDS = 1600;
    public static double RIGHT_SERVO_OPEN_MICROSECONDS = 2400;
    public static double RIGHT_SERVO_CLOSED_TELEOP_MICROSECONDS =  1200; // VALUE FOR AUTO ONLY: increase to 850 if the servo is under intensive use
    public static double RIGHT_SERVO_DROP_MICROSECONDS = 1800;
    public static double LEFT_SERVO_CLOSED_AUTO_MICROSECONDS = 2250;
    public static double RIGHT_SERVO_CLOSED_AUTO_MICROSECONDS = 1150;

    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro

}
