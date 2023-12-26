package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.controller.PIDFController;

public class PositionConfig {
    public static double xP = 0.105;
    public static double xD = 0.0175;

    public static double yP = 0.105;
    public static double yD = 0.0175;

    public static double hP = 1.5;
    public static double hD = 0.075;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public static double ALLOWED_HEADING_ERROR = 0.03;
}
