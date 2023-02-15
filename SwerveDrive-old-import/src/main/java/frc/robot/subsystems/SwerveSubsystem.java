package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
   
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveDriveOdometry odometer;

    public final SwerveModule frontLeft = new SwerveModule(
		SwerveConstants.kFrontLeftDriveMotorPort,
		SwerveConstants.kFrontLeftTurningMotorPort,
		SwerveConstants.kFrontLeftDriveEncoderReversed,
		SwerveConstants.kFrontLeftTurningEncoderReversed);

	public final SwerveModule frontRight = new SwerveModule(
		SwerveConstants.kFrontRightDriveMotorPort,
		SwerveConstants.kFrontRightTurningMotorPort,
		SwerveConstants.kFrontRightDriveEncoderReversed,
		SwerveConstants.kFrontRightTurningEncoderReversed);

	public final SwerveModule backLeft = new SwerveModule(
		SwerveConstants.kBackLeftDriveMotorPort,
		SwerveConstants.kBackLeftTurningMotorPort,
		SwerveConstants.kBackLeftDriveEncoderReversed,
		SwerveConstants.kBackLeftTurningEncoderReversed);

	public final SwerveModule backRight = new SwerveModule(
		SwerveConstants.kBackRightDriveMotorPort,
		SwerveConstants.kBackRightTurningMotorPort,
		SwerveConstants.kBackRightDriveEncoderReversed,
		SwerveConstants.kBackRightTurningEncoderReversed);



		
		
		private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

		


    public SwerveSubsystem() {
        modulePosition[0] = new SwerveModulePosition(frontLeft.driveEncoder.getPosition(), frontRight.getState().angle);
		modulePosition[1] = new SwerveModulePosition(frontRight.driveEncoder.getPosition(), frontRight.getState().angle);
		modulePosition[2] = new SwerveModulePosition(backLeft.driveEncoder.getPosition(), frontRight.getState().angle);
		modulePosition[3] = new SwerveModulePosition(backRight.driveEncoder.getPosition(), frontRight.getState().angle);

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



    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), Robot.modulePosition, pose);
    }
    
    @Override
    public void periodic() {
        //odometer = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics ,new Rotation2d(), Robot.modulePosition);

        //odometer.update(getRotation2d(), Robot.modulePosition);

        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
        


        if(desiredStates[0].speedMetersPerSecond < 10){
        } else {
            desiredStates[0].speedMetersPerSecond = 0;
        }

        if(desiredStates[1].speedMetersPerSecond < 10){
        } else {
            desiredStates[1].speedMetersPerSecond = 0;
        }

        if(desiredStates[2].speedMetersPerSecond < 10){
        } else {
            desiredStates[2].speedMetersPerSecond = 0;
        }

        if(desiredStates[3].speedMetersPerSecond < 10){
        } else {
            desiredStates[3].speedMetersPerSecond = 0;
        }


        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}