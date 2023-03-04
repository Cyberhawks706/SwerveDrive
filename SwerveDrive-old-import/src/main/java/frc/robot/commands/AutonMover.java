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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import java.lang.Math;

// class to move the robot in autonomous
public class AutonMover extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private static float currentPitch;
    public static boolean reachedRamp = false;
    public static int reachedLevel = 0;
    // constructor with input for swerve subsystem
    public AutonMover(SwerveSubsystem swerveSubsystem) {
        // add subsystem requirements
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
    // x is fwd/back
        double targetDistance = findClosest()[0];
        double targetAngle = findClosest()[1];
        
        Transform3d target;
        if(targetDistance > 120){
            target = new Transform3d(new Translation3d(0, -targetDistance/300, 0), new Rotation3d(0,0,-targetAngle));
        } else{
            //new Translation3d(0, (-targetDistance+25)/500, 0)
            target = new Transform3d(new Translation3d(), new Rotation3d(0,0,-targetAngle));
        }
        ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }
    

    private double[] findClosest() {
        double output[] = {0,0};
        double lowestDist = 1000;
        String closest = "";
        for(String subtable : table.getSubTable("processed0").getSubTables()) {
            double dist = table.getSubTable("processed0").getSubTable(subtable).getEntry("distance").getDouble(0);
            if(dist < lowestDist && dist != 0) {
                lowestDist = dist;
                closest = subtable;
            }
        }
        lowestDist = lowestDist > 999 ? 0 : lowestDist;
        output[0] = lowestDist;
        output[1] = table.getSubTable("processed0").getSubTable(closest).getEntry("xCenter").getDouble(0)-320;
        output[1] /= 1000;
        return output;
    }

    private void balance() {
        Transform3d target = calculateTargetForBalance();
        ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }



    private Transform3d calculateTargetForBalance() {
        currentPitch = swerveSubsystem.gyro.getPitch();
        Transform3d target = new Transform3d();
        if(!reachedRamp && Math.abs(currentPitch) < 10) {
            target = new Transform3d(new Translation3d(0,1.5,0), new Rotation3d());
        } else {
            reachedRamp = true;
        }
        if(reachedRamp && reachedLevel < Constants.Auton.balanceReverseDelay) {
            target = new Transform3d(new Translation3d(0,-currentPitch/15,0), new Rotation3d()); 
            if(currentPitch >5) { //higher=stop earlier
                target = new Transform3d(new Translation3d(0,-currentPitch/20,0), new Rotation3d(0,0,0));
                reachedLevel++;
            }
        } else if(reachedRamp && reachedLevel < Constants.Auton.balanceReverseDelay + 5) {
            target = new Transform3d(new Translation3d(), new Rotation3d(0,0,5));
            reachedLevel++;
        }
        //System.out.println(currentYaw);
        SmartDashboard.putNumber("pitch", currentPitch);
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