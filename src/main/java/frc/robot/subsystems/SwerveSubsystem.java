package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Swerve.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

    private static SwerveDrivePoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();

    private final SwerveDriveKinematics kDriveKinematics;

    private PhotonCameraWrapper[] cameras;

    public final static SwerveModule frontLeft = new SwerveModule(
            kFrontLeftDriveMotorPort,
            kFrontLeftTurningMotorPort,
            kFrontLeftDriveEncoderReversed,
            kFrontLeftTurningEncoderReversed,
            kFrontLeftDriveAbsoluteEncoderPort,
            kFrontLeftDriveAbsoluteEncoderOffsetrad,
            kFrontLeftDriveAbsoluteEncoderReversed);

    public final static SwerveModule frontRight = new SwerveModule(
            kFrontRightDriveMotorPort,
            kFrontRightTurningMotorPort,
            kFrontRightDriveEncoderReversed,
            kFrontRightTurningEncoderReversed,
            kFrontRightDriveAbsoluteEncoderPort,
            kFrontRightDriveAbsoluteEncoderOffsetrad,
            kFrontRightDriveAbsoluteEncoderReversed);

    public final static SwerveModule backLeft = new SwerveModule(
            kBackLeftDriveMotorPort,
            kBackLeftTurningMotorPort,
            kBackLeftDriveEncoderReversed,
            kBackLeftTurningEncoderReversed,
            kBackLeftDriveAbsoluteEncoderPort,
            kBackLeftDriveAbsoluteEncoderOffsetrad,
            kBackLeftDriveAbsoluteEncoderReversed);

    public final static SwerveModule backRight = new SwerveModule(
            kBackRightDriveMotorPort,
            kBackRightTurningMotorPort,
            kBackRightDriveEncoderReversed,
            kBackRightTurningEncoderReversed,
            kBackRightDriveAbsoluteEncoderPort,
            kBackRightDriveAbsoluteEncoderOffsetrad,
            kBackRightDriveAbsoluteEncoderReversed);

    public SwerveSubsystem(PhotonCameraWrapper... cameras) {
        updatePositions();
        this.cameras = cameras;
        kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL,FR,BL,BR
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, getRotation2d(), modulePosition, getPose());
        SmartDashboard.putData("Field", m_field);
        recenter();

    }

    public SwerveDriveKinematics getKinematics() {
        return kDriveKinematics;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), modulePosition, pose);
    }

    public void recenter() {
        resetOdometry(new Pose2d());
        zeroHeading();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * move the robot
     * 
     * @param xSpeed forwards speed, positive is away from our alliance wall
     * @param ySpeed sideways speed, positive is left
     * @param rot rotation speed, positive is counterclockwise
     * @param fieldRelative whether the xSpeed and ySpeed are relative to the field
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(moduleStates);
    }

    @Override
    public void periodic() {
        updatePositions();
        poseEstimator.update(getRotation2d(), modulePosition);
        for (PhotonCameraWrapper camera : cameras) {
            Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(getPose());
            if (result.isPresent()) {
                poseEstimator.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
            }
        }
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    private void updatePositions() {
        modulePosition[0] = frontLeft.getPosition();
        modulePosition[1] = frontRight.getPosition();
        modulePosition[2] = backLeft.getPosition();
        modulePosition[3] = backRight.getPosition();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

    }

    public void lockModules() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public Command lockModulesCommand() {
        return new RepeatCommand(new InstantCommand(() -> lockModules())).withTimeout(1).andThen(this::stopModules);
    }

    // Assuming this method is part of a drivetrain subsystem that provides the
    // necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(kPYController, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        this // Requires this drive subsystem
                ),
                new InstantCommand(() -> stopModules()));
    }

}