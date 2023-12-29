package org.firstinspires.ftc.teamcode.vision.processors;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PropDetectorConfig {
    public static int RECT_WIDTH = 125;
    public static int RECT_HEIGHT = 125;

    public static double BLUE_H_MIN = 90.0;
    public static double BLUE_S_MIN = 100.0;
    public static double BLUE_V_MIN = 100.0;

    public static double BLUE_H_MAX = 150.0;
    public static double BLUE_S_MAX = 255.0;
    public static double BLUE_V_MAX = 255.0;
    public static int BLUE_LEFT_X = 800;
    public static int BLUE_LEFT_Y = 550;
    public static int BLUE_CENTER_X = 1175;
    public static int BLUE_CENTER_Y = 175;

    public static double RED_H_MIN = 160.0;
    public static double RED_S_MIN = 100.0;
    public static double RED_V_MIN = 100.0;
    public static double RED_H_MAX = 180.0;
    public static double RED_S_MAX = 255.0;
    public static double RED_V_MAX = 255.0;
    public static int RED_LEFT_X = 900;
    public static int RED_LEFT_Y = 525;
    public static int RED_CENTER_X = 1325;
    public static int RED_CENTER_Y = 100;

    public static int COUNT_THRESHOLD = 300;
}
