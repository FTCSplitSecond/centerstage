package org.firstinspires.ftc.teamcode.drone_launcher.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DroneConfig {
    public static double HELD_MICROSECONDS = 600.0;
    public static double RELEASE_MICROSECONDS = 1000.0;
    public static double STOWED = 500;
    public static double LAUNCH = 1000;
    public static double estimatedTimeToComplete = 100; // 100 ms based on https://axon-robotics.com/products/micro
}