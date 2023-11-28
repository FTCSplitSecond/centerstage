package org.firstinspires.ftc.teamcode.swerve.subsystems;

import com.acmerobotics.dashboard.config.Config;

/// Configuration for the swerve drive
/// This is a class so that we can use the @Config annotation to make it easy to change
/// the values in the dashboard.
///
/// Note that this is a java (not kotlin) class because I couldn't get the @Config annotation
/// to work in kotlin.
@Config
public class SwerveDriveConfiguration {
    public static double wheelRevsPerMotorRev = 16.0 / 60.0 * 24.0 / 36.0 * 14.0 / 28.0;
    public static double wheelDiameter = 72.0 / 1000.0; // 72mm in meters
    public static double wheelCircumference = wheelDiameter * Math.PI;  // meters/wheel rev
    public static double getWheelMetersPerTick(double driveMotorCPR) {
        return wheelCircumference * wheelRevsPerMotorRev / driveMotorCPR;
    }
    public static double maxMotorRevs = 5000.0/60.0; // 5000 RPM in revs/second
    public static double axonMaxPlusAt6VMaxRotationalSpeed = (60.0/360.0) * 2 * Math.PI / 0.115;  // spec is 0.115 secs/60 degrees converted to rad/sec
    public static double maxAngularSpeed = axonMaxPlusAt6VMaxRotationalSpeed;
    public static double maxWheelSpeed = maxMotorRevs * wheelRevsPerMotorRev * wheelCircumference; // meters/second
}
