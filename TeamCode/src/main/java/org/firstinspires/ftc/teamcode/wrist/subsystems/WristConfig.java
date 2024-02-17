package org.firstinspires.ftc.teamcode.wrist.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WristConfig {

        public static double WRIST_SERVO_ZERO_POSITION = 1496.0; // zero degrees equals straight out
        public static double WRIST_EXTENDED_INTAKE = 0.0; //  degrees
        public static double WRIST_CLOSE_INTAKE = 0.0; //   degrees
        public static double WRIST_TRAVEL = 45.0; //  degrees
        public static double WRIST_PREDEPOSIT = -30.0;
        public static double WRIST_MOVE_OFFSET = 30; //degrees
        public static double WRIST_MAX = 90.0; //  degrees
        public static double WRIST_MIN = -30.0; //  degrees
        public static double WRIST_OFFSET = 6.0; // degrees

}
