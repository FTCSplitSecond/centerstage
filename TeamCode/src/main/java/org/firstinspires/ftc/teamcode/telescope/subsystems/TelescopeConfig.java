package org.firstinspires.ftc.teamcode.telescope.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TelescopeConfig {
    public static double TELESCOPE_KP = 3.0;
    public static double TELESCOPE_KI = 0.0;
    public static double TELESCOPE_KD = 0.04;

    public static double TELESCOPE_EXTENDED_INTAKE = 17.0; //  inches
    public static double TELESCOPE_CLOSE_INTAKE = 1.0; //   inches
    public static double TELESCOPE_DEPOSIT = 15.0; //  inches
    public static double TELESCOPE_DEPOSIT_SAFE = 4.0; //  inches
    public static double TELESCOPE_TRAVEL = 2.0; //  inches
    public static double TELESCOPE_MAX = 17.0; //  inches
    public static double TELESCOPE_MIN = 0.0; //  inches
}
