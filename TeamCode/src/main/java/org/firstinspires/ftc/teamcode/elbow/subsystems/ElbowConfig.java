package org.firstinspires.ftc.teamcode.elbow.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElbowConfig {

    public static double ELBOW_PID_KP = 0.017;
    public static double ELBOW_PID_KI = 0.02;
    public static double ELBOW_PID_KD = 0.0;
    public static double ELBOW_PID_LP_HZ = 3.0;
    public static double ELBOW_PID_ANTI_WINDUP_LIMIT = 0.1;  // the absolute max value of the integral term. 0.0 effectively disables the KI term

    public static double ELBOW_MAX = 190.0; //  degrees
    public static double ELBOW_MIN = -10.0; //  degrees
    public static double ELBOW_HOME = -8.0; //  degrees
    public static double ELBOW_EXTENDED_INTAKE = -4.0; //  degrees
    public static double ELBOW_CLOSE_INTAKE = -10.0; //   degrees
    public static double ELBOW_TRAVEL = 0.0; //  degrees
    public static double CYCLE_DROP = 165.0;
    public static double ELBOW_STACK_INTAKE = -3.0;

    public static double ELBOW_STACK_INTAKE_CLOSE = -2.25;
    public static double KS_VOLTS = 0.2; // 0.106 * 12.0; // Volts to overcome static friction (measured 0.19V but this causes instability)
    public static double KK_VOLTS = 0.0; // Volts to overcome kinetic friction
    public static double KS_OMEGA_THRESHOLD = 1.0; // angular v in deg/sec to transition from Static to Kinetic friction
    public static double KS_POWER_THRESHOLD = 0.15;

    // these multipliers are used for testing/troubleshooting/tuning in dashboard
    public static double KA_MULTIPLIER = 1.0; // multiplier for derived Ka
    public static double KV_MULTIPLIER = 1.0; // multiplier for derived Kv
    public static double KG_MULTIPLIER = 1.0; // multiplier for derived Kg
    public static double B_VISCOUS_DAMPING_COEFFICIENT = 0.60; // N*m / (rad/sec)

    public static double ELBOW_MAX_ANGULAR_VELOCITY = 250.0;  // degree/sec
    public static double ELBOW_MAX_ANGULAR_ACCELERATION = 2000.0; // degrees/sec^2
    public static double ELBOW_MAX_ANGULAR_DECELERATION = 150.0; // degrees/sec^2
    public static double ELBOW_CLIMB = 90.0;
    public static double ELBOW_PURPLE_DROP = 188.0;

    public static double ELBOW_TEST_INCREMENT = 30.0;
    public static double ELBOW_TEST_TELESCOPE_INCREMENT = 1.0;


}
