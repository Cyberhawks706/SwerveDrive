package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.Timer706;

public class SwerveSubsystem extends SubsystemBase {

    public final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

    private SwerveDriveOdometry odometer;
    private final Field2d m_field = new Field2d();
    private Trajectory m_trajectory;
    public static double offset = 0;
    

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
        SmartDashboard.putData("Field", m_field);
        m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        m_field.getObject("traj").setTrajectory(m_trajectory);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

    }

    public static void zeroHeading() { 
        gyro.reset();
        offset = 180;
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
        m_field.setRobotPose(odometer.getPoseMeters());
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

        if(RobotContainer.driverJoystick.getAButtonPressed()){
            zeroHeading();
        }

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

        desiredStates[0].angle = desiredStates[0].angle.plus(Rotation2d.fromDegrees(offset));
        desiredStates[1].angle = desiredStates[1].angle.plus(Rotation2d.fromDegrees(offset));
        desiredStates[2].angle = desiredStates[2].angle.plus(Rotation2d.fromDegrees(offset));
        desiredStates[3].angle = desiredStates[3].angle.plus(Rotation2d.fromDegrees(offset));



        // desiredStates[0].speedMetersPerSecond *= 0.3;//1/(1+4*RobotContainer.driverJoystick.getRightTriggerAxis());
        // desiredStates[1].speedMetersPerSecond *= 0.3;//1/(1+4*RobotContainer.driverJoystick.getRightTriggerAxis());
        // desiredStates[2].speedMetersPerSecond *= 0.3;//1/(1+4*RobotContainer.driverJoystick.getRightTriggerAxis());
        // desiredStates[3].speedMetersPerSecond *= 0.3;//1/(1+4*RobotContainer.driverJoystick.getRightTriggerAxis());
        

            desiredStates[0].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
            desiredStates[1].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
            desiredStates[2].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
            desiredStates[3].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
        
        

        
        


        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        


        

    


    }
}