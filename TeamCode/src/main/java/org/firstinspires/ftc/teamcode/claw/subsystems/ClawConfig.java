package org.firstinspires.ftc.teamcode.claw.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConfig {

    public static double LEFT_SERVO_OPEN_MICROSECONDS = 1300;
    public static double LEFT_SERVO_CLOSED_TELEOP_MICROSECONDS = 2000; // VALUE FOR AUTO ONLY: decrease to 1900 if the servo is under intensive use
     public static double LEFT_SERVO_DROP_MICROSECONDS = 1650;
    public static double RIGHT_SERVO_OPEN_MICROSECONDS = 1500.0;
    public static double RIGHT_SERVO_CLOSED_TELEOP_MICROSECONDS =  550.0; // VALUE FOR AUTO ONLY: increase to 850 if the servo is under intensive use
    public static double RIGHT_SERVO_DROP_MICROSECONDS = 1150;
    public static double LEFT_SERVO_CLOSED_AUTO_MICROSECONDS = 1900.0;
    public static double RIGHT_SERVO_CLOSED_AUTO_MICROSECONDS = 850.0;

    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro

}
