package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.DoubleStream;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static java.lang.Math.abs;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class CenterstageMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10.0, 0, 1.0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8.0, 0, 0.15);
    public double IMU_OFFSET = 0.0;
    public Pose2d startPose = new Pose2d();

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public TrajectorySequenceRunner trajectorySequenceRunner;


    private static final double wheelbase = 350.0/25.4; //mm to inched
    private static final double trackwidth = 292.275/25.4; //mm to inches
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, trackwidth);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private HardwareMap hardwareMap;
    private final double wheelBaseX = wheelbase / 2.0;
    private final double trackWidthY = trackwidth / 2.0;
    private final Translation2d frontLeftLocation = new Translation2d(wheelBaseX, trackWidthY);
    private final Translation2d frontRightLocation = new Translation2d(wheelBaseX, -trackWidthY);
    private final Translation2d backLeftLocation = new Translation2d(-wheelBaseX, trackWidthY);
    private final Translation2d backRightLocation = new Translation2d(-wheelBaseX, -trackWidthY);

    //Create the FTCLib MecanumDriveKinematics object with the above wheel geometry
    private final MecanumDriveKinematics mecanumDriveKinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


    public CenterstageMecanumDrive(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d());
    }

    public CenterstageMecanumDrive(HardwareMap hardwareMap, Pose2d startPose) {
        super(kV, kA, kStatic, trackwidth, wheelbase, LATERAL_MULTIPLIER);

        this.hardwareMap = hardwareMap;

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(DriveConstants.PID_TOLERANCE, DriveConstants.PID_TOLERANCE, Math.toRadians(1.0)), 1.0);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftRear = hardwareMap.get(DcMotorEx.class, "bL");
        rightRear = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // REVERSE MOTORS HERE
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this, startPose));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

//    public void setStartPose(Pose2d startPose) {
//        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this, startPose));
//    }

//    public void setStartPose(Pose2d startPose) {
//        getLocalizer().setPoseEstimate(startPose);
//    }
//
//    public Pose2d rotatePose(Pose2d pose) {
//        Vector2d vec = pose.vec();
//        Vector2d rotatedVec = vec.rotated(pose.getHeading());
//        return new Pose2d(rotatedVec.getX(), rotatedVec.getY(), pose.getHeading());
//    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    private Double getOdoHeading() {
        return getPoseEstimate().getHeading();
    }
    public Gyrodometry gyrodometry = new Gyrodometry(this::getRawExternalHeading, this::getOdoHeading);
    public void update() {
        updatePoseEstimate();
     //   gyrodometry.updateForGryrodometry();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }
    public void setWeightedDrivePower(Pose2d drivePower) {
        setWeightedDrivePower(drivePower, new Translation2d(0.0, 0.0));
    }
    public void setWeightedDrivePower(Pose2d drivePower, Translation2d centerOfRotation) {
        Pose2d vel = drivePower;

        if (abs(drivePower.getX()) + abs(drivePower.getY())
                + abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * abs(drivePower.getX())
                    + VY_WEIGHT * abs(drivePower.getY())
                    + OMEGA_WEIGHT * abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel, centerOfRotation);
    }



    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        setDrivePower(drivePower, new Translation2d(0,0));
    }

    public void setDrivePower(Pose2d drivePower, Translation2d centerOfRotation) {
        double scaleFactor = Math.sqrt(2.0); // exists here to make the ftclib kinematics work as RR kinematics
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                drivePower.getX() * scaleFactor,
                drivePower.getY()* scaleFactor,
                drivePower.getHeading()* scaleFactor);
        MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = normalizeWheelSpeedsToPowers(
                mecanumDriveKinematics.toWheelSpeeds(chassisSpeeds, centerOfRotation));

        setMotorPowers(
                adjustPowerForKStatic(mecanumDriveWheelSpeeds.frontLeftMetersPerSecond),
                adjustPowerForKStatic(mecanumDriveWheelSpeeds.rearLeftMetersPerSecond),
                adjustPowerForKStatic(mecanumDriveWheelSpeeds.rearRightMetersPerSecond),
                adjustPowerForKStatic(mecanumDriveWheelSpeeds.frontRightMetersPerSecond));
    }
    public Double adjustPowerForKStatic(Double basePower) {
        double adjustedPower = 0.0;
        double epsilon = 1e-10;
        if (Math.abs(basePower) > epsilon)  adjustedPower =  Math.signum(basePower) * kStatic + basePower;
        return Math.min(Math.max(adjustedPower, -1.0), 1.0);
    }
    public MecanumDriveWheelSpeeds normalizeWheelSpeedsToPowers(MecanumDriveWheelSpeeds wheelSpeeds) {
        double maxAbsSpeed = DoubleStream.of(wheelSpeeds.frontLeftMetersPerSecond,
                        wheelSpeeds.frontRightMetersPerSecond, wheelSpeeds.rearLeftMetersPerSecond, wheelSpeeds.rearRightMetersPerSecond)
                .map(Math::abs).max().getAsDouble();

        if (maxAbsSpeed > 1.0) {
            return new MecanumDriveWheelSpeeds(wheelSpeeds.frontLeftMetersPerSecond/maxAbsSpeed,
            wheelSpeeds.frontRightMetersPerSecond/maxAbsSpeed,
            wheelSpeeds.rearLeftMetersPerSecond/maxAbsSpeed,
            wheelSpeeds.rearRightMetersPerSecond/maxAbsSpeed);
        } else return wheelSpeeds;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public double getExternalHeadingWithOffset() {
        return AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - IMU_OFFSET);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
