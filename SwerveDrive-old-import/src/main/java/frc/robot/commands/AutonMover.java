package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import java.lang.Math;

// class to move the robot in autonomous
public class AutonMover extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private static float currentYaw;
    private static boolean reachedRamp = false;
    // constructor with input for swerve subsystem
    public AutonMover(SwerveSubsystem swerveSubsystem) {
        // add subsystem requirements
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
    // x is fwd/back
        Transform3d target = calculateTargetForBalance();
        ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }

    private Transform3d calculateTargetForBalance() {
        currentYaw = swerveSubsystem.gyro.getYaw();
        Transform3d target = new Transform3d();
        if(!reachedRamp && currentYaw < 10) {
            target = new Transform3d(new Translation3d(0,4,0), new Rotation3d());
        } else {
            reachedRamp = true;
        }
        if(reachedRamp) {
            target = new Transform3d(new Translation3d(0,currentYaw/25,0), new Rotation3d());
        }
        return target;
    }

    public void gotoTag(int camNum, int tagNum) {
        Transform3d target = calculateTagPos(camNum,tagNum);
        ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        if(target.getY() > 1) { 
            swerveSubsystem.setModuleStates(swerveModuleStates);
        }
    }

    private Transform3d calculateTagPos(int camId, int tagId) {
        String tagNum = String.valueOf(tagId);
        String camNum = String.valueOf(camId);
        System.out.print(table.getSubTable("processed0").getSubTable("tag3").getEntry("tx").getString(""));
        double x = Double.valueOf(table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("tx").getString(""));
        double y = Double.valueOf(table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("ty").getString(""));
        double z = Double.valueOf(table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("tz").getString(""));
        //double yaw = table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("yaw").getDouble(0.0);
        double yaw = 0;
        z--;
        Transform3d target = new Transform3d(new Translation3d(z,y,x), new Rotation3d(0,0,yaw));
        System.out.println(target);
        return target;
    }

    private ChassisSpeeds calculateSpeedsToTarget(Transform3d target) {
        double angularDistance = target.getRotation().getZ();
        //double rotMultiplier = Math.abs(angularDistance / SwerveConstants.kMaxAngularSpeedRadiansPerSecond);

        double xSpeed = Math.min(target.getX(), SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        double ySpeed = Math.min(target.getY(), SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        double turnSpeed = Math.min(angularDistance, SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        return chassisSpeeds;
        
    }

    public Transform3d robotFieldRelativePos(int tagId) {
        Transform3d robotPos, tagPos, tagRelativePos;
        tagPos = calculateTagPos(0, tagId);
        tagRelativePos = Constants.Auton.fieldRelativeTagPositions[tagId];
        robotPos = tagPos.plus(tagRelativePos.inverse());
        return robotPos;
    }
}