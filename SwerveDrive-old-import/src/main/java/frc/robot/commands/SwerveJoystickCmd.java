package frc.robot.commands;


import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalOutput;
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
    public static SwerveModuleState[] moduleStates;
    public static ChassisSpeeds chassisSpeeds;
    

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

        

        


        double xSpeed = -RobotContainer.driverJoystick.getLeftX();
        double ySpeed = RobotContainer.driverJoystick.getLeftY();
        double turningSpeed = -RobotContainer.driverJoystick.getRightX();
        double rSpeed = RobotContainer.manipulatorJoystick.getRightY();
        double fSpeed = RobotContainer.manipulatorJoystick.getLeftY();
        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition();
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition();
        

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
        fSpeed = Math.abs(fSpeed) > 0.1 ? fSpeed : 0.0;
        rSpeed = Math.abs(rSpeed) > 0.1 ? rSpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > SwerveConstants.kDeadband ? turningSpeed : 0.0;
        // 3. Make the driving smoother
        xSpeed *= SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

       


        //Timmy Code DANGER DANGER DANGEr!!!!!!!!!!!
        //DANGER Timmy Code!!!!!!!!!!!

        double clawTiltMotorPos = Components.sparkClawTilt.encoder.getPosition();
        double clawTiltSpeed = -RobotContainer.manipulatorJoystick.getLeftTriggerAxis() + RobotContainer.manipulatorJoystick.getRightTriggerAxis();
        double rearPotPos = Components.rearLiftPot.get();
        double frontPotPos = Components.frontLiftPot.get();
        double desiredClawPos = 0;
        double desiredFLift = 0; //Desired position for preset positions
        double desiredRLift = 0;
        

         if(RobotContainer.manipulatorJoystick.getAButton()){   // Ground Pickup Position
           desiredClawPos = 11.3;//12.619
           desiredRLift = 2.945;//2.919
           desiredFLift = 1.515;

           fSpeed = frontPotPos - desiredFLift;
           rSpeed = rearPotPos - desiredRLift;

            if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if(Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if(Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;


           clawTiltSpeed = desiredClawPos - clawTiltMotorPos;

           if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
            clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);

        }

       else if(RobotContainer.manipulatorJoystick.getYButton()){   // Human Pickup Position
            desiredClawPos = 16.833;
            desiredRLift = 2.888;
            desiredFLift = 2.735;
 
            fSpeed = frontPotPos - desiredFLift;
            rSpeed = rearPotPos - desiredRLift;
 
            if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if(Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;
 
            if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if(Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;
 
 
            clawTiltSpeed = desiredClawPos - clawTiltMotorPos;
 
            if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
                clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
 
         }

        else if(RobotContainer.manipulatorJoystick.getRawButton(7)){   // Safe Drive Position BACK BUTTON
            desiredClawPos = 0.3;
            desiredRLift = 3.1;
            desiredFLift = 0.44;
 
            fSpeed = frontPotPos - desiredFLift;
            rSpeed = rearPotPos - desiredRLift;
 
            if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
            fSpeed = fSpeed / Math.abs(fSpeed);
        else if(Math.abs(fSpeed) < 0.02)
            fSpeed /= 10;

        if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
            rSpeed = rSpeed / Math.abs(rSpeed);
        else if(Math.abs(rSpeed) < 0.02)
            rSpeed /= 10;
 
 
            clawTiltSpeed = desiredClawPos - clawTiltMotorPos;
 
            if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
                clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
 
 
         }

         else if(RobotContainer.manipulatorJoystick.getXButton()){   // Top Cone Score
            desiredClawPos = 19.428;
            desiredRLift = 2.99;
            desiredFLift = 2.955;
 
            fSpeed = frontPotPos - desiredFLift;
            rSpeed = rearPotPos - desiredRLift;
 
            if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
            fSpeed = fSpeed / Math.abs(fSpeed);
        else if(Math.abs(fSpeed) < 0.02)
            fSpeed /= 10;

        if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
            rSpeed = rSpeed / Math.abs(rSpeed);
        else if(Math.abs(rSpeed) < 0.02)
            rSpeed /= 10;

            if(frontPotPos > 2.45)
                clawTiltSpeed = desiredClawPos - clawTiltMotorPos;
 
            if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
                clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
 
         }

         else if(RobotContainer.manipulatorJoystick.getBButton()){   // Top Cone Score
            desiredClawPos = 12;//18.3
            desiredRLift = 3.19;//2.69
            desiredFLift = 3.021;//2.528
 
            fSpeed = frontPotPos - desiredFLift;
            rSpeed = rearPotPos - desiredRLift;
 
            if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if(Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;
 
            if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if(Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;
 
            if(frontPotPos > 2)
                clawTiltSpeed = desiredClawPos - clawTiltMotorPos;
 
            if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
                clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
 
         }

       
        
        fLiftMotorPos -= fSpeed*40;
        rLiftMotorPos -= rSpeed*40;

        if( ((clawTiltMotorPos + clawTiltSpeed * 0.5) > 0.43 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) < clawTiltMotorPos ||((clawTiltMotorPos + clawTiltSpeed * 0.5) < 20.75 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) > clawTiltMotorPos )))
        clawTiltMotorPos += clawTiltSpeed * 1;

        double intakeSpeed = 0;
        if(RobotContainer.manipulatorJoystick.getRightBumper())
            intakeSpeed = 1;
        if(RobotContainer.manipulatorJoystick.getLeftBumper())
            intakeSpeed = -1;

        Components.sparkClawTilt.setPos(clawTiltMotorPos);
        Components.sparkIntake.setPower(intakeSpeed);
        System.out.println("CLAW: " + Components.sparkClawTilt.encoder.getPosition());

        //End Timmy Code
        //Back to Safety!!!!!!!!!

        // 4. Construct desired chassis speeds
       
        System.out.println("r" + Components.rearLiftPot.get());
        System.out.println(Components.frontLiftPot.get());
        

        

        if (fieldOrientedFunction.get()) {
            // Relative to field
            //System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

            //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        } 
        else {
            // Relative to robot
            //System.out.println("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            //chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }
        
        
        

        // 5. Convert chassis speeds to individual module states
        moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
        Components.sparkLiftF.setPos(fLiftMotorPos);
        Components.sparkLiftR.setPos(rLiftMotorPos);
        //System.out.println(fLiftMotorPos);
        //System.out.println(rLiftMotorPos);


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