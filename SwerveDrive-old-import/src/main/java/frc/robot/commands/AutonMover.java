package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants;
import java.lang.Math;

// class to move the robot in autonomous
public class AutonMover extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    // constructor with input for swerve subsystem
    public AutonMover(SwerveSubsystem swerveSubsystem) {
        // add subsystem requirements
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {

        Transform3d target = new Transform3d();

        ChassisSpeeds chassisSpeeds = calculateSpeedsToPoint(target);
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }

    public ChassisSpeeds calculateSpeedsToPoint(Transform3d target) {
        double angularDistance = target.getRotation().getZ();
        double rotMultiplier = 1.5 * Math.abs(angularDistance / SwerveConstants.kMaxAngularSpeedRadiansPerSecond);

        double xSpeed = Math.min(target.getX(), SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        double ySpeed = Math.min(target.getY(), SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        double turnSpeed = Math.min(angularDistance*rotMultiplier, SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        return chassisSpeeds;
        
    }
}