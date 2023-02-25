package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs

        //double xSpeed = xSpdFunction.get();
        //double ySpeed = ySpdFunction.get();
        //double turningSpeed = turningSpdFunction.get();





        double xSpeed = RobotContainer.driverJoystick.getLeftX();
        double ySpeed = -RobotContainer.driverJoystick.getLeftY();
        double turningSpeed = RobotContainer.driverJoystick.getRightX();
        double fSpeed = RobotContainer.manipulatorJoystick.getRightY();
        double rSpeed = RobotContainer.manipulatorJoystick.getLeftY();
        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition();
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition();
        

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > SwerveConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > SwerveConstants.kDeadband ? ySpeed : 0.0;
        fSpeed = Math.abs(fSpeed) > SwerveConstants.kDeadband ? fSpeed : 0.0;
        rSpeed = Math.abs(rSpeed) > SwerveConstants.kDeadband ? rSpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > SwerveConstants.kDeadband ? turningSpeed : 0.0;
        // 3. Make the driving smoother
        xSpeed *= SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        fLiftMotorPos -= fSpeed*10;
        rLiftMotorPos -= rSpeed*10;


        //Timmy Code DANGER DANGER DANGEr!!!!!!!!!!!
        //DANGER Timmy Code!!!!!!!!!!!

        double fClawTiltMotorPos = Components.sparkClawTilt.encoder.getPosition();


        //End Timmy Code
        //Back to Safety!!!!!!!!!

        // 4. Construct desired chassis speeds
       
        ChassisSpeeds chassisSpeeds;

        
        
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

            //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        } 
        else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        
        

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
        Components.sparkLiftF.setPos(fLiftMotorPos);
        Components.sparkLiftR.setPos(rLiftMotorPos);


    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}