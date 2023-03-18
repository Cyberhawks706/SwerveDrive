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

    public static boolean OverrideClaw = false;

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

        double rSpeed = 0;
        double xSpeed = RobotContainer.driverJoystick.getLeftX();
        double ySpeed = -RobotContainer.driverJoystick.getLeftY();
        double turningSpeed = RobotContainer.driverJoystick.getRightX();
        if (RobotContainer.manipulatorJoystick.getRawButton(10)) {
            rSpeed = RobotContainer.manipulatorJoystick.getRightY();
        }
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
        xSpeed *= SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond / 10;
        ySpeed *= SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond / 10;
        turningSpeed *= 0.1;

        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // Timmy Code DANGER DANGER DANGEr!!!!!!!!!!!
        // DANGER Timmy Code!!!!!!!!!!!

        double clawTiltMotorPos = Components.sparkClawTilt.encoder.getPosition();
        double clawTiltPos = clawTiltMotorPos;
        double clawTiltSpeed = -RobotContainer.manipulatorJoystick.getLeftTriggerAxis()
                + RobotContainer.manipulatorJoystick.getRightTriggerAxis();
        double rearPotPos = Components.rearLiftPot.get();
        double frontPotPos = Components.frontLiftPot.get();
        double desiredClawPos = 0;
        double desiredFLift = 0; // Desired position for preset positions
        double desiredRLift = 0;

        if (RobotContainer.manipulatorJoystick.getAButton()) { // Ground Pickup Position
            desiredClawPos = 11.5;// 12.619
            desiredRLift = 2.945;// 2.919
            desiredFLift = 1.555;

            fSpeed = 1 * (frontPotPos - desiredFLift); // front number changes deceleration rate (Higher is quicker)
            rSpeed = 1 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.2)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.2)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;

            clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getYButton()) { // Human Pickup Position
            desiredClawPos = 17;
            desiredRLift = 2.888;
            desiredFLift = 2.79;

            fSpeed = 1.5 * (frontPotPos - desiredFLift);
            rSpeed = 1.5 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.2)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.2)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;

            clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getRawButton(7)) { // Safe Drive Position BACK BUTTON
            desiredClawPos = 5;
            desiredRLift = 3.1;
            desiredFLift = 1;

            fSpeed = 1 * (frontPotPos - desiredFLift);
            rSpeed = 1 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.2)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.2)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;

            if (frontPotPos < 1.25)
                desiredClawPos = 0.5;

            clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getXButton()) { // Top Cone Score
            desiredClawPos = 19.26;
            desiredRLift = 2.99;
            desiredFLift = 2.955;

            fSpeed = 2 * (frontPotPos - desiredFLift);
            rSpeed = 2 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.05)
                fSpeed /= 20;
            else if (Math.abs(fSpeed) < 0.1)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.4)
                fSpeed = 0.4 * (fSpeed / Math.abs(fSpeed));

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.05)
                rSpeed /= 20;
            else if (Math.abs(rSpeed) < 0.1)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.4)
                rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));

            if (frontPotPos > 2.45)
                clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getBButton()) { // Mid Cone Score
            desiredClawPos = 18.3;// 18.3
            desiredRLift = 2.69;// 2.69
            desiredFLift = 2.528;// 2.528

            fSpeed = 1.5 * (frontPotPos - desiredFLift);

            if (frontPotPos > 1.5)
                rSpeed = 1.5 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.2)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.2)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;

            if (frontPotPos > 2)
                clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getPOV() == 90) { // Mid Cube Score
            desiredClawPos = 12;// 18.3
            desiredRLift = 2.69;// 2.69
            desiredFLift = 2.528;// 2.528

            fSpeed = 1.5 * (frontPotPos - desiredFLift);

            if (frontPotPos > 1.5)
                rSpeed = 1.5 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.2)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.2)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;

            if (frontPotPos > 2)
                clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getPOV() == 270) { // Top Cube Score
            desiredClawPos = 12;// 18.3
            desiredRLift = 3.19;// 2.69
            desiredFLift = 3.021;// 2.528

            fSpeed = 1.5 * (frontPotPos - desiredFLift);
            rSpeed = 1.5 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.2)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.02)
                fSpeed /= 10;

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.2)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.02)
                rSpeed /= 10;

            if (frontPotPos > 2.45)
                clawTiltPos = desiredClawPos;

        }

        // if(frontPotPos > 3.02 || frontPotPos < 0.05){
        // System.out.println("LIMIT REACHED");
        // } else {
        // fLiftMotorPos -= fSpeed*40;
        // rLiftMotorPos -= rSpeed*40;
        // }

        // fLiftMotorPos -= fSpeed*0.25;

        // rLiftMotorPos -= rSpeed*0.25;

        if (RobotContainer.manipulatorJoystick.getRawButton(10)) {
            rLiftMotorPos -= rSpeed * 0.25;
        }

        if (RobotContainer.manipulatorJoystick.getRawButton(8)) {
            OverrideClaw = true;
        } else {
            OverrideClaw = false;
        }

        if (!OverrideClaw) {
            if (((clawTiltMotorPos + clawTiltSpeed * 0.5) > 0.43
                    && (clawTiltMotorPos + clawTiltSpeed * 0.5) < clawTiltMotorPos
                    || ((clawTiltMotorPos + clawTiltSpeed * 0.5) < 19.3
                            && (clawTiltMotorPos + clawTiltSpeed * 0.5) > clawTiltMotorPos)))
                clawTiltMotorPos += clawTiltSpeed * 6;
        }

        double intakeSpeed = 0;
        if (RobotContainer.manipulatorJoystick.getRightBumper())
            intakeSpeed = 1;
        if (RobotContainer.manipulatorJoystick.getLeftBumper())
            intakeSpeed = -1;

        Components.sparkClawTilt.setPos(clawTiltPos);
        Components.sparkIntake.setPower(intakeSpeed);

        if (RobotContainer.driverJoystick.getAButtonPressed()) {
            System.out.println("GRYRO IS BEING RREESSEETT");
            SwerveSubsystem.zeroHeading();
        }
        // System.out.println("CLAW: " +
        // Components.sparkClawTilt.encoder.getPosition());

        // End Timmy Code
        // Back to Safety!!!!!!!!!

        // 4. Construct desired chassis speeds

        // System.out.println("r" + Components.rearLiftPot.get());
        // System.out.println(Components.frontLiftPot.get());
        // Components.sparkClawTilt.UpdateSensorValues();
        // System.out.println(Components.sparkClawTilt.motorPos);

        if (fieldOrientedFunction.get()) {
            // Relative to field
            // System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                    swerveSubsystem.getRotation2d());

            // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        } else {
            // Relative to robot
            // System.out.println("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
            // turningSpeed, swerveSubsystem.getRotation2d());
        }

        // 5. Convert chassis speeds to individual module states
        moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        if (Math.abs(fSpeed) < 0.1)
            Components.sparkLiftF.setPos(fLiftMotorPos);
        else
            Components.sparkLiftF.setPower(-fSpeed);
        if (Math.abs(rSpeed) < 0.1)
            Components.sparkLiftR.setPos(rLiftMotorPos);
        else
            Components.sparkLiftR.setPower(-rSpeed * 0.5);

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