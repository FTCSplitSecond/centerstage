package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -1.9; // X is the up and down direction
    public static double PARALLEL_Y = 6.0; // Y is the strafe direction

    public static double PERPENDICULAR_X = 5.0;
    public static double PERPENDICULAR_Y = 3.4;

    public static double X_MULTIPLIER = 0.9942;

    public static double Y_MULTIPLIER = 0.9975;


    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis

    private Encoder parallelEncoder, perpendicularEncoder;

    private CenterstageMecanumDrive drive;
    private double correctionFromImuToFieldHeading = 0.0;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, CenterstageMecanumDrive drive, Pose2d startPose) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fL"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bL"));

        updateIMUHeadingCorrection(startPose.getHeading());
        setPoseEstimate(startPose);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d value) {
        super.setPoseEstimate(value);
        updateIMUHeadingCorrection(value.getHeading());
    }

    public void updateIMUHeadingCorrection(Double newHeading) {
        correctionFromImuToFieldHeading = AngleUnit.normalizeRadians(newHeading - drive.getRawExternalHeading());
    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading() + correctionFromImuToFieldHeading;
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition())*X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())*Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity())*X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())*Y_MULTIPLIER
        );
    }
}
