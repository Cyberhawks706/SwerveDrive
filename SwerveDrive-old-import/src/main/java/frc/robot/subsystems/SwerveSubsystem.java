package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
   
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveDriveOdometry odometer;


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
        odometer.resetPosition(getRotation2d(), Robot.modulePosition, pose);
    }
    
    @Override
    public void periodic() {
        odometer = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics ,new Rotation2d(), Robot.modulePosition);

        odometer.update(getRotation2d(), Robot.modulePosition);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        Robot.frontLeft.stop();
        Robot.frontRight.stop();
        Robot.backLeft.stop();
        Robot.backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Math.IEEEremainder(gyro.getAngle(), 360));
        Robot.frontLeft.setDesiredState(desiredStates[0]);
        Robot.frontRight.setDesiredState(desiredStates[1]);
        Robot.backLeft.setDesiredState(desiredStates[2]);
        Robot.backRight.setDesiredState(desiredStates[3]);
    }
}