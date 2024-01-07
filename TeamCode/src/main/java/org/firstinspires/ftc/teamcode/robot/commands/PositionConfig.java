package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
@Config
public class PositionConfig {
    public static double xP = 0.06;
    public static double xD = 0;

    public static double yP = 0.03;
    public static double yD = 0;

    public static double hP = 0.55;
    public static double hD = 0;

    public static double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public static double ALLOWED_HEADING_ERROR = 0.03;

    public static double X = 12.0;

    public static double Y = 12.0;

    public static double H = Math.PI/2.0;
}
