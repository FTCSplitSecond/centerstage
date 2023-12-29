package org.firstinspires.ftc.teamcode.elbow.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElbowConfig {

    public static double ELBOW_KP = 0.15;
    public static double ELBOW_KI = 0.0;
    public static double ELBOW_KD = 0.004;
    public static double ELBOW_MAX = 160.0; //  degrees
    public static double ELBOW_MIN = -7.0; //  degrees
    public static double ELBOW_HOME = -7.0; //  degrees
    public static double ELBOW_EXTENDED_INTAKE = -1.0; //  degrees
    public static double ELBOW_CLOSE_INTAKE = -5.5; //   degrees
    public static double ELBOW_DEPOSIT = 139.0; //  degrees
    public static double ELBOW_DEPOSIT_SAFE = 128.0; //  degrees
    public static double ELBOW_TRAVEL = 18.0; //  degrees




}
