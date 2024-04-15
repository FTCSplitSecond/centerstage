package org.firstinspires.ftc.teamcode.elbow.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElbowConfig {

    public static double ELBOW_KP = 0.02;
    public static double ELBOW_KI = 0.0;
    public static double ELBOW_KD = 0.0015;
    public static double ELBOW_MAX = 190.0; //  degrees
    public static double ELBOW_MIN = -10.0; //  degrees
    public static double ELBOW_HOME = -8.0; //  degrees
    public static double ELBOW_EXTENDED_INTAKE = -4.0; //  degrees
    public static double ELBOW_CLOSE_INTAKE = -10.0; //   degrees
    public static double ELBOW_TRAVEL = 0.0; //  degrees
    public static double CYCLE_DROP = 155.0;
    public static double ELBOW_STACK_INTAKE = -3.7;
    public static double ELBOW_STACK_INTAKE_CLOSE = -1.2;
    public static double KS_VOLTS = 0.19; // Volts to overcome static friction
    public static double KK_VOLTS = 0.0; // Volts to overcome kinetic friction
    public static double KS_OMEGA_THRESHOLD = 0.001; // angular v in rad/sec to transition from Static to Kinetic friction

    public static double KA_MULTIPLIER = 0.0; // multiplier for derived Ka
    public static double KV_MULTIPLIER = 0.0; // multiplier for derived Kv
    public static double KG_MULTIPLIER = 1.0; // multiplier for derived Kg

    public static double B_VISCOUS_DAMPING = 0.0;

    public static double ELBOW_MAX_ANGULAR_VELOCITY = 800.0;  // degree/sec
    public static double ELBOW_MAX_ANGULAR_ACCELERATION = 800.0; // degrees/sec
    public static double ELBOW_CLIMB = 90.0;
    public static double ELBOW_PURPLE_DROP = 188.0;

    public static double ELBOW_TEST_INCREMENT = 5.0;
    public static double ELBOW_TEST_TELESCOPE_INCREMENT = 1.0;


}
