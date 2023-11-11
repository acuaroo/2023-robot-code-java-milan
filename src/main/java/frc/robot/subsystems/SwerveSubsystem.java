package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controllers.SwerveModuleControlller;
import frc.robot.utils.NetworkTableUtils;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.VisionUtils;

import java.util.Arrays;

public class SwerveSubsystem extends SubsystemBase {
    // Defining Motors
    private final SwerveModuleControlller frontLeft = new SwerveModuleControlller(
            DrivetrainConstants.frontLeftDrivingPort,
            DrivetrainConstants.frontLeftTurningPort,
            DrivetrainConstants.frontLeftChassisAngularOffset
    );

    private final SwerveModuleControlller frontRight = new SwerveModuleControlller(
            DrivetrainConstants.frontRightDrivingPort,
            DrivetrainConstants.frontRightTurningPort,
            DrivetrainConstants.frontRightChassisAngularOffset
    );

    private final SwerveModuleControlller rearLeft = new SwerveModuleControlller(
            DrivetrainConstants.rearLeftDrivingPort,
            DrivetrainConstants.rearLeftTurningPort,
            DrivetrainConstants.rearLeftChassisAngularOffset
    );

    private final SwerveModuleControlller rearRight = new SwerveModuleControlller(
            DrivetrainConstants.rearRightDrivingPort,
            DrivetrainConstants.rearRightTurningPort,
            DrivetrainConstants.rearRightChassisAngularOffset
    );

    // Gyro
    private final AHRS gyro = new AHRS();

    // Slew Rate Constants
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate Limiters
    private final SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DrivetrainConstants.magnitudeSlewRate);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(DrivetrainConstants.rotationalSlewRate);

    // Slew Rate Time
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    // Limelight Network Table
    private final NetworkTableUtils limelightTable = new NetworkTableUtils("limelight");

    // Convert Gyro angle to radians(-2pi to 2pi)
    public double heading() {
        return Units.degreesToRadians(-1 * (gyro.getAngle() + 180.0) % 360.0);
    }

    // Swerve Odometry
//    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
//            DrivetrainConstants.driveKinematics,
//            Rotation2d.fromRadians(heading()),
//            new SwerveModulePosition[]{
//                    frontLeft.getPosition(),
//                    frontRight.getPosition(),
//                    rearLeft.getPosition(),
//                    rearRight.getPosition()
//            }
//    );

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.driveKinematics,
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // Network Tables Telemetry
    private final DoubleArrayEntry setpointsTelemetry = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleArrayTopic("Setpoints").getEntry(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    private final DoubleArrayEntry actualTelemetry = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleArrayTopic("Actual").getEntry(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

//    private final DoubleArrayEntry poseTelemetry = NetworkTableInstance.getDefault()
//            .getTable("Swerve").getDoubleArrayTopic("Pose").getEntry(new double[]{odometry.getPoseMeters().getTranslation().getX(),
//                    odometry.getPoseMeters().getTranslation().getY(),
//                    odometry.getPoseMeters().getRotation().getRadians()});

    private final DoubleEntry gyroHeading = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("GyroHeading").getEntry(heading());

    private final DoubleEntry frontrightpos = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("frpos").getEntry(frontRight.getPosition().angle.getRadians());

    private final DoubleEntry frontleftpos = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleTopic("flpos").getEntry(frontLeft.getPosition().angle.getRadians());

    private final DoubleArrayEntry visPoseEstimator = NetworkTableInstance.getDefault()
            .getTable("Swerve").getDoubleArrayTopic("VisPose").getEntry(new double[]{poseEstimator.getEstimatedPosition().getX(),
                    poseEstimator.getEstimatedPosition().getY(),
                    poseEstimator.getEstimatedPosition().getRotation().getRadians()});
    // Periodic
    @Override
    public void periodic() {
        // Update odometry
//        odometry.update(
//                poseEstimator.getEstimatedPosition().getRotation(),
//                new SwerveModulePosition[]{
//                        frontLeft.getPosition(),
//                        frontRight.getPosition(),
//                        rearLeft.getPosition(),
//                        rearRight.getPosition()
//                }
//        );


        Pose2d visionPos = VisionUtils.getBotPose2d_wpiBlue("");

        if (visionPos.getX() != 0 && visionPos.getY() != 0) {
            poseEstimator.addVisionMeasurement(
                    visionPos,
                    Timer.getFPGATimestamp() - (VisionUtils.getLatency_Pipeline("")/1000.0) - (VisionUtils.getLatency_Capture("")/1000.0)
            );
        }

        poseEstimator.update(
                new Rotation2d(heading()),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                }
        );

        visPoseEstimator.set(new double[]{
                poseEstimator.getEstimatedPosition().getX(),
                poseEstimator.getEstimatedPosition().getY(),
                poseEstimator.getEstimatedPosition().getRotation().getRadians(),
        });

//        System.out.println(poseEstimator.getEstimatedPosition().getX());
        System.out.println(poseEstimator.getEstimatedPosition().getY());
//        System.out.println(poseEstimator.getEstimatedPosition().getX());

        // Coen's Vision Lineup Thing:
        // find the botpose network table id thingy, construct a pose2d, feed it into resetodometry
//        double[] botpose = limelightTable.getDoubleArray("botpose", new double[0]);
//        if (!Arrays.equals(botpose, new double[0])) {
//            Pose2d pose = new Pose2d(new Translation2d(botpose[0], botpose[2]), new Rotation2d(botpose[3], botpose[5]));
//            resetOdometry(pose);
//        }

        frontrightpos.set(frontRight.getPosition().angle.getRadians());
        frontleftpos.set(frontLeft.getPosition().angle.getRadians());

        // Set Network Tables Telemetry
        actualTelemetry.set(new double[]{
                frontLeft.getPosition().angle.getRadians(), frontLeft.getState().speedMetersPerSecond,
                frontRight.getPosition().angle.getRadians(), frontRight.getState().speedMetersPerSecond,
                rearLeft.getPosition().angle.getRadians(), rearLeft.getState().speedMetersPerSecond,
                rearRight.getPosition().angle.getRadians(), rearRight.getState().speedMetersPerSecond});

        setpointsTelemetry.set(new double[]{
                frontLeft.getDesiredState().angle.getRadians(), frontLeft.getDesiredState().speedMetersPerSecond,
                frontRight.getDesiredState().angle.getRadians(), frontRight.getDesiredState().speedMetersPerSecond,
                rearLeft.getDesiredState().angle.getRadians(), rearLeft.getDesiredState().speedMetersPerSecond,
                rearRight.getDesiredState().angle.getRadians(), rearRight.getDesiredState().speedMetersPerSecond});

//        poseTelemetry.set(new double[]{
//                odometry.getPoseMeters().getTranslation().getX(),
//                odometry.getPoseMeters().getTranslation().getY(),
//                odometry.getPoseMeters().getRotation().getRadians()
//        });

        gyroHeading.set(heading());
    }

    // Define robot pose
    private Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // Reset odometry function
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                Rotation2d.fromRadians(heading()),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                },
                pose
        );
    }

    // Drive function - slew rate limited to prevent shearing of wheels
    public void drive(double forwardMetersPerSecond, double sidewaysMetersPerSecond, double radiansPerSecond, boolean fieldRelative, boolean rateLimit) {
        // forward is xspeed, sideways is yspeed
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            double inputTranslationDirection = Math.atan2(sidewaysMetersPerSecond, forwardMetersPerSecond);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(forwardMetersPerSecond, 2.0) + Math.pow(sidewaysMetersPerSecond, 2.0));

            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DrivetrainConstants.directionSlewRate / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // super high number means slew is instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;

            double angleDifference = SwerveUtils.AngleDifference(inputTranslationDirection, currentTranslationDirection);
            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                        currentTranslationDirection,
                        inputTranslationDirection,
                        directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude > 1e-4) { // small number avoids floating-point errors
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                        currentTranslationDirection,
                        inputTranslationDirection,
                        directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            }

            previousTime = currentTime;

            xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(radiansPerSecond);
        } else {
            xSpeedCommanded = forwardMetersPerSecond;
            ySpeedCommanded = sidewaysMetersPerSecond;
            currentRotation = radiansPerSecond;
        }

        double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.maxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.maxSpeedMetersPerSecond;
        double rotationDelivered = currentRotation * DrivetrainConstants.maxAngularSpeed;

        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeedDelivered,
                            ySpeedDelivered,
                            rotationDelivered,
                            Rotation2d.fromRadians(heading())
                    )
            );
        } else {
            swerveModuleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Sets the wheels to an X configuration
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    }

    // Sets the wheels to a zeroed configuration
    public void setZero() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    // Resets Gyro
    public void zeroGyro() {
        gyro.reset();
    }

    // Resets Gyro and odometry
    public void zeroGyroAndOdometry() {
        gyro.reset();
        resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    // Sets states of swerve modules
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    // Resets Swerve encoders
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
    }
}