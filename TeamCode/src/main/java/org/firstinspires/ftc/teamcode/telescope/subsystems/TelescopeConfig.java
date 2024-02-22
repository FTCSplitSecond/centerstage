package org.firstinspires.ftc.teamcode.telescope.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TelescopeConfig {
    public static double TELESCOPE_KP = 0.5
            ;
    public static double TELESCOPE_KI = 0.0;
    public static double TELESCOPE_KD = 0.025;
    public static double TELESCOPE_MAX = 19.0; //  inches
    public static double TELESCOPE_MIN = 0.0; //  inches
    public static double TELESCOPE_EXTENDED_INTAKE = TELESCOPE_MAX - 0.75; //  inches

    // These seems to "feel better" when out in front of the claw a bit
    public static double TELESCOPE_EXTENDED_INTAKE_COR_X = 32.0; //  inches from center of robot to center of rotation
    public static double TELESCOPE_CLOSE_INTAKE_COR_X = 0.0; //  inches from center of robot to center of rotation
    public static double TELESCOPE_DEPOSIT_COR_X_OFFSET = -3.0; //  inches from center of robot to center of rotation
    public static double TELESCOPE_CLOSE_INTAKE = 1.0; //   inches

    public static double TELESCOPE_MAX_ACCELERATION = 10.0; //inches per second per second
    public static double TELESCOPE_MAX_VELOCITY = 10.0; // inches per second

    public static double TELESCOPE_ADJUST = 0.0; // inches

    public static double TELESCOPE_TRAVEL = 0.0; //  inches
    public static double TELESCOPE_CLIMB = 18.0; //  inches
    public static double IK_X_OFFSET = 0.0; //inches

}