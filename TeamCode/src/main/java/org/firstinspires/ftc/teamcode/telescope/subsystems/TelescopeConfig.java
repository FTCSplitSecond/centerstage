package org.firstinspires.ftc.teamcode.telescope.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TelescopeConfig {
    public static double TELESCOPE_KP = 0.75;
    public static double TELESCOPE_KI = 0.0;
    public static double TELESCOPE_KD = 0.0;
    public static double TELESCOPE_MAX = 16.0; //  inches
    public static double TELESCOPE_MIN = 0.0; //  inches
    public static double TELESCOPE_EXTENDED_INTAKE = TELESCOPE_MAX - 0.75; //  inches
    public static double TELESCOPE_CLOSE_INTAKE = 1.0; //   inches

    public static double TELESCOPE_TRAVEL = 3.0; //  inches
    public static double IK_OFFSET_CONFIG = -1.0; //inches

}