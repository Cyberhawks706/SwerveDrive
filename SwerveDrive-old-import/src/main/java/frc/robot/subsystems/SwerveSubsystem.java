package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends SubsystemBase {

    public final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

    private SwerveDriveOdometry odometer;
    public final Field2d m_field = new Field2d();
    public static double offset = 0;

    private final SwerveDriveKinematics kDriveKinematics;

    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;    

    public boolean firstTime = true;
    public boolean reachedFrontRight = false;
    public boolean reachedFrontLeft = false;
    public boolean reachedBackLeft = false;
    public boolean reachedBackRight = false;


    public final static SwerveModule frontLeft = new SwerveModule(
		Constants.Swerve.kFrontLeftDriveMotorPort,
		Constants.Swerve.kFrontLeftTurningMotorPort,
		Constants.Swerve.kFrontLeftDriveEncoderReversed,
		Constants.Swerve.kFrontLeftTurningEncoderReversed,
        Constants.Swerve.kFrontLeftDriveAbsoluteEncoderPort,
        Constants.Swerve.kFrontLeftDriveAbsoluteEncoderOffsetrad,
        Constants.Swerve.kFrontLeftDriveAbsoluteEncoderReversed);

	public final static SwerveModule frontRight = new SwerveModule(
		Constants.Swerve.kFrontRightDriveMotorPort,
		Constants.Swerve.kFrontRightTurningMotorPort,
		Constants.Swerve.kFrontRightDriveEncoderReversed,
		Constants.Swerve.kFrontRightTurningEncoderReversed,
        Constants.Swerve.kFrontRightDriveAbsoluteEncoderPort,
        Constants.Swerve.kFrontRightDriveAbsoluteEncoderOffsetrad,
        Constants.Swerve.kFrontRightDriveAbsoluteEncoderReversed);

	public final static SwerveModule backLeft = new SwerveModule(
		Constants.Swerve.kBackLeftDriveMotorPort,
		Constants.Swerve.kBackLeftTurningMotorPort,
		Constants.Swerve.kBackLeftDriveEncoderReversed,
		Constants.Swerve.kBackLeftTurningEncoderReversed,
        Constants.Swerve.kBackLeftDriveAbsoluteEncoderPort,
        Constants.Swerve.kBackLeftDriveAbsoluteEncoderOffsetrad,
        Constants.Swerve.kBackLeftDriveAbsoluteEncoderReversed);

	public final static SwerveModule backRight = new SwerveModule(
		Constants.Swerve.kBackRightDriveMotorPort,
		Constants.Swerve.kBackRightTurningMotorPort,
		Constants.Swerve.kBackRightDriveEncoderReversed,
		Constants.Swerve.kBackRightTurningEncoderReversed,
        Constants.Swerve.kBackRightDriveAbsoluteEncoderPort,
        Constants.Swerve.kBackRightDriveAbsoluteEncoderOffsetrad,
        Constants.Swerve.kBackRightDriveAbsoluteEncoderReversed);

    public SwerveSubsystem() {

        updatePositions();
        kDriveKinematics  = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL,FR,BL,BR
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), modulePosition);
        
        SmartDashboard.putData("Field", m_field);
        
        xLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        turnLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond); 


    }

    public SwerveDriveKinematics getKinematics() {
        return kDriveKinematics;
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
    public Pose2d getPose() {
        return odometer.getPoseMeters();
        //return new Pose2d(odometer.getPoseMeters().getTranslation().times(-3).rotateBy(Rotation2d.fromDegrees(-90)), odometer.getPoseMeters().getRotation());
    }

    /* 
     * move the robot
     * @param xSpeed forwards speed, positive is away from our alliance wall
     * @param ySpeed sideways speed, positive is left
     * @param rot rotation speed, positive is counterclockwise
    */
    public void drive(double xSpeed, double ySpeed, double rot) {
        //limit acceleration
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        rot = turnLimiter.calculate(rot);
        
        //deadband
        if (Math.abs(xSpeed) < kDeadband) xSpeed = 0;
        if (Math.abs(ySpeed) < kDeadband) ySpeed = 0;
        if (Math.abs(rot) < kDeadband) rot = 0;
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rot, getRotation2d()));
        
        setModuleStates(moduleStates);
    }

    public void teleDrive(double xSpeed, double ySpeed, double rot, double accelMultiplier) {
        ySpeed *= kTeleDriveMaxSpeedMetersPerSecond * (accelMultiplier + 0.11);
        xSpeed *= kTeleDriveMaxSpeedMetersPerSecond * (accelMultiplier + 0.11);
        rot *= kTeleDriveMaxAngularSpeedRadiansPerSecond * (accelMultiplier + 0.11);
        drive(xSpeed, ySpeed, rot);
    }

    @Override
    public void periodic() {
        updatePositions();
        
        odometer.update(getRotation2d(), modulePosition);
        m_field.setRobotPose(getPose());
        // System.out.println(getPose());
        //System.out.println(frontLeft.driveEncoder.getPosition());
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

    public void robotAlign(SwerveModuleState[] desiredStates){
        if(!reachedFrontRight){
            desiredStates[1].angle = Rotation2d.fromRadians(frontRight.initPos);
            desiredStates[1].angle = desiredStates[1].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kFrontRightDriveAbsoluteEncoderOffsetrad));
            frontRight.setDesiredState(desiredStates[1]);
            frontRight.resetEncoders();
            reachedFrontRight = true;
        }
            
        if(!reachedFrontLeft){
            desiredStates[0].angle = Rotation2d.fromRadians(frontLeft.initPos);
            desiredStates[0].angle = desiredStates[0].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kFrontLeftDriveAbsoluteEncoderOffsetrad));
            frontLeft.setDesiredState(desiredStates[0]);
            frontLeft.resetEncoders();
            reachedFrontLeft = true;
        }
            
        if(!reachedBackLeft){
            desiredStates[2].angle = Rotation2d.fromRadians(backLeft.initPos);
            desiredStates[2].angle = desiredStates[2].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kBackLeftDriveAbsoluteEncoderOffsetrad));
            backLeft.setDesiredState(desiredStates[2]);
            backLeft.resetEncoders();
            reachedBackLeft = true;
        }
            
        if(!reachedBackRight){
            desiredStates[3].angle = Rotation2d.fromRadians(backRight.initPos);
            desiredStates[3].angle = desiredStates[3].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kBackRightDriveAbsoluteEncoderOffsetrad));
            backRight.setDesiredState(desiredStates[3]);
            backRight.resetEncoders();
            reachedBackRight = true;
        }
            
        
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
        

        if(firstTime){
            robotAlign(desiredStates);
            firstTime = false;
        }

        desiredStates[0].angle = desiredStates[0].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kFrontLeftDriveAbsoluteEncoderOffsetrad));
        desiredStates[1].angle = desiredStates[1].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kFrontRightDriveAbsoluteEncoderOffsetrad));
        desiredStates[2].angle = desiredStates[2].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kBackLeftDriveAbsoluteEncoderOffsetrad));
        desiredStates[3].angle = desiredStates[3].angle.plus(Rotation2d.fromRadians(Constants.Swerve.kBackRightDriveAbsoluteEncoderOffsetrad));
        
        for (SwerveModuleState desiredState : desiredStates) {
            if(desiredState.speedMetersPerSecond > 20) {
                desiredState.speedMetersPerSecond = 0;
            }
            desiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(90));
            desiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(offset));
        }

        // desiredStates[0].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
        // desiredStates[1].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
        // desiredStates[2].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);
        // desiredStates[3].speedMetersPerSecond *= 16*(RobotContainer.driverJoystick.getRightTriggerAxis()+0.11);


        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);


    }
    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                kDriveKinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }

    public Command xboxDriveCommand(CommandXboxController controller) {
        return this.run(
						() -> this.teleDrive(
								controller.getLeftY(),
								controller.getLeftX(),
								controller.getRightX(),
								controller.getRightTriggerAxis()));
    }
}