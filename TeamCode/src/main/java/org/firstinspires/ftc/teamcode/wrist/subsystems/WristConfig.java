package org.firstinspires.ftc.teamcode.wrist.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WristConfig {

        public static double LEFT_SERVO_ZERO_POSITION = 1496.0; // zero degrees equals straight out
        public static double RIGHT_SERVO_ZERO_POSITION = 1390.0; // zero degrees equals straight out

        public static double WRIST_EXTENDED_INTAKE = 3.0; //  degrees
        public static double WRIST_CLOSE_INTAKE = 6.0; //   degrees
        public static double WRIST_TRAVEL = 90.0; //  degrees
        public static double WRIST_MAX = 90.0; //  degrees
        public static double WRIST_MIN = -30.0; //  degrees

}
