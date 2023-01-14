package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components;
import frc.robot.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            SwerveConstants.kFrontLeftDriveMotorPort,
            SwerveConstants.kFrontLeftTurningMotorPort,
            SwerveConstants.kFrontLeftDriveEncoderReversed,
            SwerveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            SwerveConstants.kFrontRightDriveMotorPort,
            SwerveConstants.kFrontRightTurningMotorPort,
            SwerveConstants.kFrontRightDriveEncoderReversed,
            SwerveConstants.kFrontRightTurningEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            SwerveConstants.kBackLeftDriveMotorPort,
            SwerveConstants.kBackLeftTurningMotorPort,
            SwerveConstants.kBackLeftDriveEncoderReversed,
            SwerveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            SwerveConstants.kBackRightDriveMotorPort,
            SwerveConstants.kBackRightTurningMotorPort,
            SwerveConstants.kBackRightDriveEncoderReversed,
            SwerveConstants.kBackRightTurningEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics ,new Rotation2d(), null);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }



    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), null, pose);
    }
    
    @Override
    public void periodic() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

         modulePosition[0] = new SwerveModulePosition(frontLeft.driveEncoder.getPosition(), frontRight.getState().angle);
         modulePosition[1] = new SwerveModulePosition(frontRight.driveEncoder.getPosition(), frontRight.getState().angle);
         modulePosition[2] = new SwerveModulePosition(backLeft.driveEncoder.getPosition(), frontRight.getState().angle);
         modulePosition[3] = new SwerveModulePosition(backRight.driveEncoder.getPosition(), frontRight.getState().angle);

        odometer.update(getRotation2d(), modulePosition);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Math.IEEEremainder(gyro.getAngle(), 360));
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}