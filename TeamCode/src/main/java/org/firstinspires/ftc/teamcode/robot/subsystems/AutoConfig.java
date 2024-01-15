package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoConfig {
    // BLUE CONSTANTS

    public static int MODE = 0;
    public static double BACKDROP_Y = -19.0;
    public static double BACKDROP_CENTER_X = -27.0;
    public static double BACKDROP_RIGHT_X = -33.0;
    public static double BACKDROP_LEFT_X = -19.0;

    public static double FAR_LANE_X = -49.75;
    public static double FAR_LANE_START_Y = 4.0;
    public static double FAR_LANE_END_Y = -67.5;

    public static double CLOSE_PARK_X = 0.0;
    public static double CLOSE_PARK_Y = 0.0;

    public static double FAR_PARK_X = 0.0;
    public static double FAR_PARK_Y = 0.0;

    public static double[] BLUE_CLOSE_CENTER_X = new double[]{-24.0, -36.0, BACKDROP_CENTER_X};
    public static double[] BLUE_CLOSE_RIGHT_X = new double[]{-28.0, -28.0, BACKDROP_RIGHT_X};
    public static double[] BLUE_CLOSE_LEFT_X = new double[]{-24.0, -25.0, BACKDROP_LEFT_X};

    public static double[] BLUE_CLOSE_CENTER_Y = new double[]{-11.0, -11.0, BACKDROP_Y};
    public static double[] BLUE_CLOSE_RIGHT_Y = new double[]{0.0, 3.0, BACKDROP_Y};
    public static double[] BLUE_CLOSE_LEFT_Y = new double[]{-24.0, -20.0, BACKDROP_Y};

    public static double[] BLUE_FAR_CENTER_X = new double[]{-24.0, -36.0, -33.0};
    public static double[] BLUE_FAR_RIGHT_X = new double[]{-9.75, -14.75, BACKDROP_RIGHT_X};
    public static double[] BLUE_FAR_LEFT_X = new double[]{-24.0, -25.0, BACKDROP_LEFT_X};

    // TODO: Complete center and finish adding paths
    public static double[] BLUE_FAR_CENTER_Y = new double[]{4.0, -67.75, FAR_LANE_START_Y};

    // TODO: Far side left and right
    public static double[] BLUE_FAR_RIGHT_Y = new double[]{4.0, 8.0, BACKDROP_Y};
    public static double[] BLUE_FAR_LEFT_Y = new double[]{-16.0, -16.0, BACKDROP_Y};

    public static double[] RED_CLOSE_CENTER_X = new double[]{-24.0, -38.0, -26.0};
    public static double[] RED_CLOSE_CENTER_Y = new double[]{14.0, 14.0, 32.5};

    public static double[] RED_CLOSE_RIGHT_X = new double[]{0.0, 0.0, 0.0};
    public static double[] RED_CLOSE_RIGHT_Y = new double[]{0.0, 0.0, 0.0};

    public static double[] RED_FAR_CENTER_X = new double[]{0.0, -50.0, -50.0};
    public static double[] RED_FAR_CENTER_Y = new double[]{-6.0, -6.0, 75.75};

    public static double[] RED_FAR_LEFT_X = new double[]{0.0, -50.0, -50.0};
    public static double[] RED_FAR_LEFT_Y = new double[]{-16.0, -16.0, 75.75};

}
