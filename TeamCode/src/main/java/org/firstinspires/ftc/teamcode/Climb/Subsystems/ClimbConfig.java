package org.firstinspires.ftc.teamcode.Climb.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClimbConfig {
    public static double LEFT_SERVO_HELD_MICROSECONDS = 650.0;
    public static double LEFT_SERVO_UP_MICROSECONDS = 1550.0;
    public static double RIGHT_SERVO_HELD_MICROSECONDS = 2500.0;
    public static double RIGHT_SERVO_UP_MICROSECONDS = 1550.0;
    public static double SUSPEND_ROBOT = 2500.0;
    public static double NEUTRAL_POS = 1500.0;

    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro

}
