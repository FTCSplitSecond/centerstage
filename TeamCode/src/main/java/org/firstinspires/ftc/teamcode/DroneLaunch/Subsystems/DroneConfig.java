package org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DroneConfig {
    public static double HELD_MICROSECONDS = 500.0;
    public static double LAUNCH_MICROSECONDS = 2500.0;
    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro
}