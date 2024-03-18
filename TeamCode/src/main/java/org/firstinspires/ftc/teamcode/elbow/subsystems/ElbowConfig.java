package org.firstinspires.ftc.teamcode.elbow.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElbowConfig {

    public static double ELBOW_KP = 0.04;
    public static double ELBOW_KI = 0.0;
    public static double ELBOW_KD = 0.0023;
    public static double ELBOW_MAX = 190.0; //  degrees
    public static double ELBOW_MIN = -10.0; //  degrees
    public static double ELBOW_HOME = -8.0; //  degrees
    public static double ELBOW_EXTENDED_INTAKE = -4.0; //  degrees
    public static double ELBOW_CLOSE_INTAKE = -10.0; //   degrees
    public static double ELBOW_TRAVEL = 0.0; //  degrees
    public static double CYCLE_DROP = 165.0;
    public static double ELBOW_STACK_INTAKE = -3.0;

    public static double ELBOW_STACK_INTAKE_CLOSE = -2.25;
    public static double KG = 0.1;
    public static double KS = 0.00;

    public static double ELBOW_MAX_ANGULAR_VELOCITY = 800.0;
    public static double ELBOW_MAX_ANGULAR_ACCELERATION = 600.0; // (REMEMBER TO CHANGE THIS TO 600!!!!!)
    public static double ELBOW_CLIMB = 90.0;
    public static double ELBOW_PURPLE_DROP = 188.0;

    public static double ELBOW_TEST_INCREMENT = 5.0;


}
