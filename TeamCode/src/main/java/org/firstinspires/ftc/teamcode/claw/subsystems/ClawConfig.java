package org.firstinspires.ftc.teamcode.claw.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConfig {
    public static double LEFT_SERVO_OPEN_MICROSECONDS = 550.0;
    public static double LEFT_SERVO_CLOSED_MICROSECONDS = 1200.0;
     public static double LEFT_SERVO_DROP_MICROSECONDS = 900.0;
    public static double RIGHT_SERVO_OPEN_MICROSECONDS = 1800.0;
    public static double RIGHT_SERVO_CLOSED_MICROSECONDS = 1175.0;
    public static double RIGHT_SERVO_DROP_MICROSECONDS = 1550.0;

    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro

}
