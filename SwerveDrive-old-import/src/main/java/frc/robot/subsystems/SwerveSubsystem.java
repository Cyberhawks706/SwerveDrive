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
import frc.robot.commands.SwerveJoystickCmd;

public class SwerveSubsystem extends SubsystemBase {

    public final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

    private SwerveDriveOdometry odometer;
    

    public boolean firstTime = true;
    public boolean reachedFrontRight = false;
    public boolean reachedFrontLeft = false;
    public boolean reachedBackLeft = false;
    public boolean reachedBackRight = false;


    public final static SwerveModule frontLeft = new SwerveModule(
		SwerveConstants.kFrontLeftDriveMotorPort,
		SwerveConstants.kFrontLeftTurningMotorPort,
		SwerveConstants.kFrontLeftDriveEncoderReversed,
		SwerveConstants.kFrontLeftTurningEncoderReversed,
        SwerveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        SwerveConstants.kFrontLeftDriveAbsoluteEncoderOffsetrad,
        SwerveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

	public final static SwerveModule frontRight = new SwerveModule(
		SwerveConstants.kFrontRightDriveMotorPort,
		SwerveConstants.kFrontRightTurningMotorPort,
		SwerveConstants.kFrontRightDriveEncoderReversed,
		SwerveConstants.kFrontRightTurningEncoderReversed,
        SwerveConstants.kFrontRightDriveAbsoluteEncoderPort,
        SwerveConstants.kFrontRightDriveAbsoluteEncoderOffsetrad,
        SwerveConstants.kFrontRightDriveAbsoluteEncoderReversed);

	public final static SwerveModule backLeft = new SwerveModule(
		SwerveConstants.kBackLeftDriveMotorPort,
		SwerveConstants.kBackLeftTurningMotorPort,
		SwerveConstants.kBackLeftDriveEncoderReversed,
		SwerveConstants.kBackLeftTurningEncoderReversed,
        SwerveConstants.kBackLeftDriveAbsoluteEncoderPort,
        SwerveConstants.kBackLeftDriveAbsoluteEncoderOffsetrad,
        SwerveConstants.kBackLeftDriveAbsoluteEncoderReversed);

	public final static SwerveModule backRight = new SwerveModule(
		SwerveConstants.kBackRightDriveMotorPort,
		SwerveConstants.kBackRightTurningMotorPort,
		SwerveConstants.kBackRightDriveEncoderReversed,
		SwerveConstants.kBackRightTurningEncoderReversed,
        SwerveConstants.kBackRightDriveAbsoluteEncoderPort,
        SwerveConstants.kBackRightDriveAbsoluteEncoderOffsetrad,
        SwerveConstants.kBackRightDriveAbsoluteEncoderReversed);

    public SwerveSubsystem() {

        modulePosition[0] = new SwerveModulePosition(frontLeft.driveEncoder.getPosition(), frontRight.getState().angle);
        modulePosition[1] = new SwerveModulePosition(frontRight.driveEncoder.getPosition(), frontRight.getState().angle);
        modulePosition[2] = new SwerveModulePosition(backLeft.driveEncoder.getPosition(), frontRight.getState().angle);
        modulePosition[3] = new SwerveModulePosition(backRight.driveEncoder.getPosition(), frontRight.getState().angle);

        odometer = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, new Rotation2d(0), modulePosition);


        
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
        return Rotation2d.fromDegrees(-getHeading());
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePosition, pose);
    }

    @Override
    public void periodic() {

        odometer.update(getRotation2d(), modulePosition);

        SmartDashboard.putNumber("Robot Heading", getHeading());

        
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void robotAllign(SwerveModuleState[] desiredStates){
        if(!reachedFrontRight){
            desiredStates[1].angle = Rotation2d.fromRadians(Robot.frontRightInitPos);
            desiredStates[1].angle = desiredStates[1].angle.plus(Rotation2d.fromRadians(SwerveConstants.kFrontRightDriveAbsoluteEncoderOffsetrad));
            frontRight.setDesiredState(desiredStates[1]);
            frontRight.resetEncoders();
            reachedFrontRight = true;
        }
            

        
        if(!reachedFrontLeft){
            desiredStates[0].angle = Rotation2d.fromRadians(Robot.frontLeftInitPos);
            desiredStates[0].angle = desiredStates[0].angle.plus(Rotation2d.fromRadians(SwerveConstants.kFrontLeftDriveAbsoluteEncoderOffsetrad));
            frontLeft.setDesiredState(desiredStates[0]);
            frontLeft.resetEncoders();
            reachedFrontLeft = true;
        }
            


        if(!reachedBackLeft){
            desiredStates[2].angle = Rotation2d.fromRadians(Robot.backLeftInitPos);
            desiredStates[2].angle = desiredStates[2].angle.plus(Rotation2d.fromRadians(SwerveConstants.kBackLeftDriveAbsoluteEncoderOffsetrad));
            backLeft.setDesiredState(desiredStates[2]);
            backLeft.resetEncoders();
            reachedBackLeft = true;
        }
            


        if(!reachedBackRight){
            desiredStates[3].angle = Rotation2d.fromRadians(Robot.backRightInitPos);
            desiredStates[3].angle = desiredStates[3].angle.plus(Rotation2d.fromRadians(SwerveConstants.kBackRightDriveAbsoluteEncoderOffsetrad));
            backRight.setDesiredState(desiredStates[3]);
            backRight.resetEncoders();
            reachedBackRight = true;
        }
            
        
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // get Front-Right Absolute Encoder Radians
        //System.out.println(frontRight.getAbsoluteEncoderRad());

        
        
        //desiredStates[0].angle = desiredStates[0].angle.minus(getRotation2d().fromRadians(Robot.frontLeftInitPos));

        
        if(firstTime){
            robotAllign(desiredStates);
            firstTime = false;
        }

        
        if(desiredStates[0].speedMetersPerSecond < 20){
        } else {
            desiredStates[0].speedMetersPerSecond = 0;
        }

        if(desiredStates[1].speedMetersPerSecond < 20){
        } else {
            desiredStates[1].speedMetersPerSecond = 0;
        }

        if(desiredStates[2].speedMetersPerSecond < 20){
        } else {
            desiredStates[2].speedMetersPerSecond = 0;
        }

        if(desiredStates[3].speedMetersPerSecond < 20){
        } else {
            desiredStates[3].speedMetersPerSecond = 0;
        }

        desiredStates[0].angle = desiredStates[0].angle.plus(Rotation2d.fromRadians(SwerveConstants.kFrontLeftDriveAbsoluteEncoderOffsetrad));
        desiredStates[1].angle = desiredStates[1].angle.plus(Rotation2d.fromRadians(SwerveConstants.kFrontRightDriveAbsoluteEncoderOffsetrad));
        desiredStates[2].angle = desiredStates[2].angle.plus(Rotation2d.fromRadians(SwerveConstants.kBackLeftDriveAbsoluteEncoderOffsetrad));
        desiredStates[3].angle = desiredStates[3].angle.plus(Rotation2d.fromRadians(SwerveConstants.kBackRightDriveAbsoluteEncoderOffsetrad));

        desiredStates[0].angle = desiredStates[0].angle.plus(Rotation2d.fromDegrees(90));
        desiredStates[1].angle = desiredStates[1].angle.plus(Rotation2d.fromDegrees(90));
        desiredStates[2].angle = desiredStates[2].angle.plus(Rotation2d.fromDegrees(90));
        desiredStates[3].angle = desiredStates[3].angle.plus(Rotation2d.fromDegrees(90));


        
        



        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        


        

    


    }
}