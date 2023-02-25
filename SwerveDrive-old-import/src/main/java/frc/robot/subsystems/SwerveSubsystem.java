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

    public final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

    private SwerveDriveOdometry odometer;

    private double a = 0;
    private double b = 0;
    private double c = 0;
    private double d = 0;

    private boolean reachedA = false;
    private boolean reachedB = false;
    private boolean reachedC = false;
    private boolean reachedD = false;


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
        modulePosition[1] = new SwerveModulePosition(frontRight.driveEncoder.getPosition(),
                frontRight.getState().angle);
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
        return Rotation2d.fromDegrees(getHeading());
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // get Front-Right Absolute Encoder Radians
        //gaer = frontRight.getAbsoluteEncoderRad();
        //System.out.println(gaer);
        //desiredStates[1].angle = Rotation2d.fromRadians(gaer);
        
        //frontRight.setDesiredState(desiredStates[1]);

        
        /*if(!reachedA && Math.abs(frontRight.getAbsoluteEncoderRad()) > 0.05){
            a += 0.01;
            desiredStates[1].angle = Rotation2d.fromRadians(a);
        } else if ( Math.abs(frontRight.getAbsoluteEncoderRad()) < 0.05){
            reachedA = true;
        }

        if(!reachedB && Math.abs(frontLeft.getAbsoluteEncoderRad()) > 0.05){
            b += 0.01;
            desiredStates[0].angle = Rotation2d.fromRadians(b);
        } else if ( Math.abs(frontLeft.getAbsoluteEncoderRad()) < 0.05){
            reachedB = true;
        }

        if(!reachedC && Math.abs(backLeft.getAbsoluteEncoderRad()) > 0.05){
            c += 0.01;
            desiredStates[2].angle = Rotation2d.fromRadians(c);
        } else if ( Math.abs(backLeft.getAbsoluteEncoderRad()) < 0.05){
            reachedC = true;
        }

        if(!reachedD && Math.abs(backRight.getAbsoluteEncoderRad()) > 0.05){
            d += 0.01;
            desiredStates[3].angle = Rotation2d.fromRadians(d);
        } else if ( Math.abs(backRight.getAbsoluteEncoderRad()) < 0.05){
            reachedD = true;
        }*/
        
        
        
        
        //desiredStates[0].angle = desiredStates[0].angle.minus(getRotation2d().fromRadians(Robot.frontLeftInitPos));
        //desiredStates[1].angle = desiredStates[1].angle.minus(getRotation2d().fromRadians(Robot.frontRightInitPos));
        //desiredStates[2].angle = desiredStates[2].angle.minus(getRotation2d().fromRadians(Robot.backLeftInitPos));
        //desiredStates[3].angle = desiredStates[3].angle.minus(getRotation2d().fromRadians(Robot.backRightInitPos));



        /*if(desiredStates[0].speedMetersPerSecond < 15){
        } else {
            desiredStates[0].speedMetersPerSecond = 0;
        }

        if(desiredStates[1].speedMetersPerSecond < 15){
        } else {
            desiredStates[1].speedMetersPerSecond = 0;
        }

        if(desiredStates[2].speedMetersPerSecond < 15){
        } else {
            desiredStates[2].speedMetersPerSecond = 0;
        }

        if(desiredStates[3].speedMetersPerSecond < 15){
        } else {
            desiredStates[3].speedMetersPerSecond = 0;
        }


        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);




        
        System.out.println(frontRight.getAbsoluteEncoderRad());*/

        //System.out.println(x);
        //System.out.println("SPARK");
        //y = frontLeft.getTurningPosition();
        //System.out.println(y);
        //System.out.println("OFFSET");
        //System.out.println(Robot.frontRightInitPos);
        //System.out.println("DIFFERENCE");
        //System.out.println(x-y);
        



        //System.out.println(frontLeft.getAbsoluteEncoderRad());
        //System.out.println(backRight.getAbsoluteEncoderRad());
        //System.out.println(backLeft.getAbsoluteEncoderRad());




    }
}