package org.firstinspires.ftc.teamcode.elbow.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElbowConfig {

    public static double ELBOW_KP = 0.0135;
    public static double ELBOW_KI = 0.0;
    public static double ELBOW_KD = 0.0006;
    public static double ELBOW_MAX = 170.0; //  degrees
    public static double ELBOW_MIN = -8.0; //  degrees
    public static double ELBOW_HOME = -8.0; //  degrees
    public static double ELBOW_EXTENDED_INTAKE = -3.0; //  degrees
    public static double ELBOW_CLOSE_INTAKE = -5.5; //   degrees
    public static double ELBOW_TRAVEL = 0.0; //  degrees
    public static double KG = 0.07;
    public static double ELBOW_MAX_ANGULAR_VELOCITY = 800.0;
    public static double ELBOW_MAX_ANGULAR_ACCELERATION = 800.0;




}
