package org.firstinspires.ftc.teamcode.claw.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConfig {
    public static double LEFT_SERVO_OPEN_MICROSECONDS = 500.0;
    public static double LEFT_SERVO_CLOSED_MICROSECONDS = 1500.0;
    public static double RIGHT_SERVO_OPEN_MICROSECONDS = 2500.0;
    public static double RIGHT_SERVO_CLOSED_MICROSECONDS = 1500.0;
    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro

}
