package org.firstinspires.ftc.teamcode.robot.util;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoConfig {
    // BLUE CONSTANTS

    public static int MODE = 2;
    public static double FAR_DELAY = 1.5;
    public static double BACKDROP_Y = -19.0;
    public static double BACKDROP_CENTER_X = -27.0;
    public static double BACKDROP_RIGHT_X = -33.0;
    public static double BACKDROP_LEFT_X = -19.0;

    public static double[] START_POSE = new double[]{-32.0, 62.0, PI/2};
    public static double[] BLUE_CLOSE_CENTER_X = new double[]{-24.0, -36.0, BACKDROP_CENTER_X};

    public static double[] BLUE_CLOSE_RIGHT_X = new double[]{-28.0, -28.0, BACKDROP_RIGHT_X};
    public static double[] BLUE_CLOSE_LEFT_X = new double[]{-24.0, -25.0, BACKDROP_LEFT_X};

    public static double[] BLUE_CLOSE_CENTER_Y = new double[]{-11.0, -11.0, BACKDROP_Y};
    public static double[] BLUE_CLOSE_RIGHT_Y = new double[]{0.0, 3.0, BACKDROP_Y};
    public static double[] BLUE_CLOSE_LEFT_Y = new double[]{-24.0, -20.0, BACKDROP_Y};

    public static double[] BLUE_FAR_CENTER_X = new double[]{-36.0, -36.0, 30.0, 30.0, 42.0, 35.0, 40.0};
    public static double[] BLUE_FAR_RIGHT_X = new double[]{-44.0, -44.0, 30.0, 30.0, 42.0, 35.0, 40.0};
    public static double[] BLUE_FAR_LEFT_X = new double[]{-36.0, -40.0, -40.0, 30.0, 30.0, 42.0, 35.0, 40.0};

    public static double[] BLUE_FAR_CENTER_Y = new double[]{62.0, 13.0, 13.0, 35.0, 35.0, 35.0, 20.0};
    public static double[] BLUE_FAR_RIGHT_Y = new double[]{62.0, 13.0, 13.0, 32.0, 32.0, 32.0, 20.0};
    public static double[] BLUE_FAR_LEFT_Y = new double[]{62.0, 30.0, 13.0, 13.0, 32.0, 32.0, 32.0, 20.0};

    public static double[] RED_CLOSE_CENTER_X = new double[]{-24.0, -38.0, -26.0};
    public static double[] RED_CLOSE_CENTER_Y = new double[]{14.0, 14.0, 32.5};

    public static double[] RED_CLOSE_RIGHT_X = new double[]{0.0, 0.0, 0.0};
    public static double[] RED_CLOSE_RIGHT_Y = new double[]{0.0, 0.0, 0.0};

    public static double[] RED_FAR_CENTER_X = new double[]{0.0, -50.0, -50.0};
    public static double[] RED_FAR_CENTER_Y = new double[]{-6.0, -6.0, 75.75};

    public static double[] RED_FAR_LEFT_X = new double[]{0.0, -50.0, -50.0};
    public static double[] RED_FAR_LEFT_Y = new double[]{-16.0, -16.0, 75.75};

}
