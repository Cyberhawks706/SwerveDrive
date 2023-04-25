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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Components;
import frc.robot.Constants;
import java.lang.Math;


// class to move the robot in autonomous
public class AutonMover extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private static float currentPitch = 0;
    public static boolean reachedRamp = false;
    public static int reachedLevel = 0;
    public static int pickupCounter = 0;
    public static boolean pickedUp = false;
    public static double startingAngle;
    public static boolean turned = false;
    public static int initialPlaced = 0;
    private static int timer = 0;

    public static int tester = 0;
    public static String position = "c"; // c for center, l for left, r for right


    // constructor with input for swerve subsystem
    public AutonMover(SwerveSubsystem swerveSubsystem) {
        // add subsystem requireme nts
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        startingAngle = swerveSubsystem.getHeading();


    }

    public static void init() {
        currentPitch = 0;
        reachedRamp = false;
        reachedLevel = 0;
        pickupCounter = 0;
        pickedUp = false;
        startingAngle = 0;
        turned = false;
        initialPlaced = 0;
    }

    @Override
    public void execute() {

        // fLiftMotorPosition = Components.sparkLiftF.encoder.getPosition();
        //double rLiftMotorPosition = Components.sparkLiftR.encoder.getPosition();
        double rearPotPos = Components.rearLiftPot.get();
        double frontPotPos = Components.frontLiftPot.get();
        double desiredClawPos;
        double desiredRLift;
        double desiredFLift;

        double fSpeed = 0;
        double rSpeed = 0;

        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition();
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition();
        double clawTiltPos = Components.sparkClawTilt.encoder.getPosition();


        

        if(timer < 50){ //bring out claw so it doesn't hit the arm
            System.out.println("SETTING POSITION");
            Components.sparkClawTilt.setPos(1.8);
        }
        
        if(timer < 450 && timer > 50){

            desiredClawPos = 17.66;
            desiredRLift = 2.82;
            desiredFLift = 2.906;

            fSpeed = 2 * (frontPotPos - desiredFLift);
            rSpeed = 2 * (rearPotPos - desiredRLift);

            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.025)
                fSpeed /= 20;
            else if (Math.abs(fSpeed) < 0.1)
                fSpeed = 0.3 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.4)
                fSpeed = 0.6 * (fSpeed / Math.abs(fSpeed));

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.05)
                rSpeed /= 20;
            else if (Math.abs(rSpeed) < 0.1)
                rSpeed = 0.3 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.4)
                rSpeed = 0.6 * (rSpeed / Math.abs(rSpeed));

            if (frontPotPos > 2.45)
                clawTiltPos = desiredClawPos;


            if (Math.abs(fSpeed) < 0.1)
                Components.sparkLiftF.setPos(fLiftMotorPos);
            else
                Components.sparkLiftF.setPower(-fSpeed);
            if (Math.abs(rSpeed) < 0.1)
                Components.sparkLiftR.setPos(rLiftMotorPos);
            else
                Components.sparkLiftR.setPower(-rSpeed * 0.5);
            Components.sparkClawTilt.setPos(clawTiltPos);
    }

        if(timer < 455 && timer > 400){
                Components.sparkIntake.setPower(-1);
        } 
        if(timer > 455 && timer < 460){
                Components.sparkIntake.setPower(0);
        }
        if(position == "c"){
           if(timer > 460 && timer < 500){

            Transform3d target1 = new Transform3d();
            target1 = new Transform3d(new Translation3d(0,-1.0,0), new Rotation3d(0,0,0));            
            ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target1);
            SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(swerveModuleStates);

            } if (timer > 500 && timer < 2000) {
                armDown();

                balance();
            }
        } else if (timer > 460 && timer < 660) { //860
                //pickupCube();
                Transform3d target1 = new Transform3d(); //-0.6
                target1 = new Transform3d(new Translation3d(0,-1,0), new Rotation3d(0,0,0));            
                ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target1);
                SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
                swerveSubsystem.setModuleStates(swerveModuleStates);


        } else if (timer > 720){
            Transform3d target1 = new Transform3d();
            ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target1);
            SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(swerveModuleStates);


        }
        
          
            

            
        timer++;
    }

    private void armDown() {
        double desiredClawPos;
        double desiredRLift;
        double desiredFLift;
        double fSpeed;
        double rSpeed;
        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition();
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition();
        double clawTiltPos = Components.sparkClawTilt.encoder.getPosition();
        double rearPotPos = Components.rearLiftPot.get();
        double frontPotPos = Components.frontLiftPot.get();
        desiredClawPos = 5;
        desiredRLift = 3.1;
        desiredFLift = 1;

        fSpeed = 1 * (frontPotPos - desiredFLift);
        rSpeed = 1 * (rearPotPos - desiredRLift);

        if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
            fSpeed = fSpeed / Math.abs(fSpeed);
        else if (Math.abs(fSpeed) < 0.025)
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

        if (frontPotPos < 1.25)
            desiredClawPos = 0.5;

        clawTiltPos = desiredClawPos;

        if (Math.abs(fSpeed) < 0.1)
            Components.sparkLiftF.setPos(fLiftMotorPos);
        else
            Components.sparkLiftF.setPower(-fSpeed);
        if (Math.abs(rSpeed) < 0.1)
            Components.sparkLiftR.setPos(rLiftMotorPos);
        else
            Components.sparkLiftR.setPower(-rSpeed * 0.5);

        Components.sparkClawTilt.setPos(clawTiltPos);
    }
        
    public void pickupCube() {
        double pickupHeading = startingAngle + 180;
        Transform3d target = new Transform3d();
        
        if (Math.abs(swerveSubsystem.getHeading() - pickupHeading) > 15 && !turned) { //we need to turn it turn it
            // if(timer < 800){
            //     target = new Transform3d(new Translation3d(0,-0.65,0), new Rotation3d(0,0,0));
            //     Components.sparkIntake.setPower(0);
            //     System.out.println(timer);
            // }   else {
            //     System.out.println("STOOOOOOP");
            //     target = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));

            // }         
            System.out.println("if #1");
            //scoreTopCone();
            System.out.println(swerveSubsystem.getHeading() - pickupHeading);
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.3, (swerveSubsystem.getHeading() - pickupHeading)/200, swerveSubsystem.getRotation2d());
            SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(swerveModuleStates);
            //timer++;
                
            return;
        } else if(!pickedUp){ //go forward
            turned = true;
            
            double targetDistance = findClosest()[0]; 
            double targetAngle = findClosest()[1];
            System.out.println("go forward" + targetAngle);
            SmartDashboard.putNumber("distance", targetDistance);
            if(targetDistance > 50) {
                System.out.println("not picked up, >36");
                target = new Transform3d(new Translation3d(0, targetDistance/100, 0), new Rotation3d(0,0,targetAngle));
                Components.sparkIntake.setPower(0);
                armDown();
            } else {
                target = new Transform3d(new Translation3d(0, 0.1,0), new Rotation3d());
                pickedUp = !Components.intakeSwitch.get();
                armDown();
            }
            ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
            SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(swerveModuleStates);
        } else if(turned && pickedUp) {
            // Components.sparkIntake.setPower(0);
            // ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
            // SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
            // swerveSubsystem.setModuleStates(swerveModuleStates);
            if (Math.abs(swerveSubsystem.getHeading() - startingAngle) > 5) { //need to turn back
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.3, (swerveSubsystem.getHeading() - startingAngle)/200, swerveSubsystem.getRotation2d());
                SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
                swerveSubsystem.setModuleStates(swerveModuleStates);
            } else {
                switch (DriverStation.getAlliance()) {
                    case Blue:
                        if(position == "l") {
                            gotoTag(0,1);
                        } else if(position == "r") {
                            gotoTag(0, 3);
                        }
                        break;
                    case Red:
                        if(position == "l") {
                            gotoTag(0,6);
                        } else if(position == "r") {
                            gotoTag(0, 8);
                        }
                        break;
                    case Invalid: break;
                }
            }
        }
        
    }

    
    public void pickup() {
        double clawTiltMotorPos = Components.sparkClawTilt.encoder.getPosition();
        double rearPotPos = Components.rearLiftPot.get();
        double frontPotPos = Components.frontLiftPot.get();
        double desiredClawPos = 13.0;//12.619
        double desiredRLift = 2.945;//2.919
        double desiredFLift = 1.615;

        double fSpeed = frontPotPos - desiredFLift;
        double rSpeed = rearPotPos - desiredRLift;

        if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
            fSpeed = fSpeed / Math.abs(fSpeed);
        else if(Math.abs(fSpeed) < 0.02)
            fSpeed /= 10;

        if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
            rSpeed = rSpeed / Math.abs(rSpeed);
        else if(Math.abs(rSpeed) < 0.02)
            rSpeed /= 10;
        
        
        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition() - fSpeed*50;
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition() - rSpeed*50;

        double clawTiltSpeed = desiredClawPos - clawTiltMotorPos;

        if(Math.abs(clawTiltSpeed) > 1) clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);

        if( ((clawTiltMotorPos + clawTiltSpeed * 0.5) > 0.43 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) < clawTiltMotorPos ||((clawTiltMotorPos + clawTiltSpeed * 0.5) < 20.75 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) > clawTiltMotorPos ))) {
            clawTiltMotorPos += clawTiltSpeed * 1; 
        }
        Components.sparkClawTilt.setPos(clawTiltMotorPos);
        Components.sparkLiftF.setPos(fLiftMotorPos);
        Components.sparkLiftR.setPos(rLiftMotorPos);
        Components.sparkIntake.setPower(1);
        pickupCounter++;
    }

    public void scoreTopCone() {
        
        double clawTiltMotorPos = Components.sparkClawTilt.encoder.getPosition();
        double rearPotPos = Components.rearLiftPot.get();
        double frontPotPos = Components.frontLiftPot.get();

       double desiredClawPos = 2;
       double desiredRLift = 2.99;
       double desiredFLift = 2.955;

       System.out.println("clawPos "+clawTiltMotorPos);


        double fSpeed = 2 * (frontPotPos - desiredFLift);
        double rSpeed = 2 * (rearPotPos - desiredRLift);

        
 
            if(Math.abs(fSpeed) > 1) //Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if(Math.abs(fSpeed) < 0.05)
                fSpeed /= 20;
            else if(Math.abs(fSpeed) < 0.1)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if(Math.abs(fSpeed) < 0.4)
                fSpeed = 0.4 * (fSpeed / Math.abs(fSpeed));
           
 
            if(Math.abs(rSpeed) > 1) //Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if(Math.abs(rSpeed) < 0.05)
                rSpeed /= 20;
            else if(Math.abs(rSpeed) < 0.1)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if(Math.abs(rSpeed) < 0.4)
                rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));
            


                double clawTiltSpeed = 0;
        if(frontPotPos > 1)
            clawTiltSpeed = desiredClawPos - clawTiltMotorPos;
        

        if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
            clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
    
            if(Math.abs(clawTiltSpeed) > 1) clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
    
            if( ((clawTiltMotorPos + clawTiltSpeed * 0.5) > 0.43 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) < clawTiltMotorPos ||((clawTiltMotorPos + clawTiltSpeed * 0.5) < 20.75 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) > clawTiltMotorPos ))) {
                clawTiltMotorPos += clawTiltSpeed * 2; 
            }

            Components.sparkClawTilt.setPos(clawTiltMotorPos);
            
            System.out.println("clawS "+clawTiltSpeed);
            System.out.println("fS "+fSpeed);
            System.out.println("rS "+rSpeed);
/*
        double fSpeed = frontPotPos - desiredFLift;
       double rSpeed = rearPotPos - desiredRLift;

        if(Math.abs(fSpeed) > 0.25) //Slow Down/Speed Up Front Arm
        fSpeed = fSpeed / Math.abs(fSpeed);
    else if(Math.abs(fSpeed) < 0.02)
        fSpeed /= 10;

    if(Math.abs(rSpeed) > 0.25) //Slow Down/Speed up Rear Arm
        rSpeed = rSpeed / Math.abs(rSpeed);
    else if(Math.abs(rSpeed) < 0.02)
        rSpeed /= 10;
    double clawTiltSpeed = 0;
        if(frontPotPos > 2.45)
            clawTiltSpeed = desiredClawPos - clawTiltMotorPos;

        if(Math.abs(clawTiltSpeed) > 1)  //Slow Down Claw
            clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);

            
    
            if(Math.abs(clawTiltSpeed) > 1) clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);
    
            if( ((clawTiltMotorPos + clawTiltSpeed * 0.5) > 0.43 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) < clawTiltMotorPos ||((clawTiltMotorPos + clawTiltSpeed * 0.5) < 20.75 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) > clawTiltMotorPos ))) {
                clawTiltMotorPos += clawTiltSpeed * 2; 
            }

            
            Components.sparkClawTilt.setPos(clawTiltMotorPos);
            
       
    
         if(Math.abs(fSpeed) < 0.1)    
        Components.sparkLiftF.setPos(fLiftMotorPos);
        else
        Components.sparkLiftF.setPower(-fSpeed);
        if(Math.abs(rSpeed) < 0.1)
        Components.sparkLiftR.setPos(rLiftMotorPos);
        else
        Components.sparkLiftR.setPower(-rSpeed*0.5);
            //Components.sparkIntake.setPower(-1); 

    

        if(Math.abs(clawTiltSpeed) < 0.75 && Math.abs(fSpeed) < 0.5 && Math.abs(rSpeed) < 0.5) {
            int x = -1;
           
        System.out.println(x);
            for(int i=0; i<=5250; i++){
                Components.sparkIntake.setPower(x); 
            }
        initialPlaced = 350;*/
        }


       /*  double desiredClawPos = 12;//18.3
        double desiredRLift = 3.19;//2.69
        double desiredFLift = 3.021;//2.528

        double fSpeed = frontPotPos - desiredFLift;
        double rSpeed = rearPotPos - desiredRLift;

        if(Math.abs(fSpeed) > 0.5) //Slow Down/Speed Up Front Arm
            fSpeed = fSpeed / Math.abs(fSpeed);
        else if(Math.abs(fSpeed) < 0.02)
            fSpeed /= 10;

        if(Math.abs(rSpeed) > 0.5) //Slow Down/Speed up Rear Arm
            rSpeed = rSpeed / Math.abs(rSpeed);
        else if(Math.abs(rSpeed) < 0.02)
            rSpeed /= 10;
        
        
        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition() - fSpeed*40;
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition() - rSpeed*40;

        double clawTiltSpeed = desiredClawPos - clawTiltMotorPos;

        if(Math.abs(clawTiltSpeed) > 1) clawTiltSpeed = clawTiltSpeed / Math.abs(clawTiltSpeed);

        if( ((clawTiltMotorPos + clawTiltSpeed * 0.5) > 0.43 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) < clawTiltMotorPos ||((clawTiltMotorPos + clawTiltSpeed * 0.5) < 20.75 &&  (clawTiltMotorPos + clawTiltSpeed * 0.5 ) > clawTiltMotorPos ))) {
            clawTiltMotorPos += clawTiltSpeed * 1; 
        }
        Components.sparkClawTilt.setPos(clawTiltMotorPos);
        Components.sparkLiftF.setPos(fLiftMotorPos);
        Components.sparkLiftR.setPos(rLiftMotorPos); 
    } 
    */
    
    
    public static double[] findClosest() {
        double output[] = {0,0};
        double lowestDist = 1000;
        String closest = "";
        for(String subtable : table.getSubTable("processed1").getSubTables()) {
            double dist = table.getSubTable("processed1").getSubTable(subtable).getEntry("distance").getDouble(0);
            if(dist < lowestDist && dist != 0) {
                lowestDist = dist;
                closest = subtable;
            }
        }
        lowestDist = lowestDist > 999 ? 0 : lowestDist;
        output[0] = lowestDist;
        output[1] = table.getSubTable("processed1").getSubTable(closest).getEntry("xCenter").getDouble(320)-320;
        output[1] /= 1000;
        //DriverStation.reportWarning(String.valueOf(lowestDist), false);
        return output;
    }

    private void balance() {
        Transform3d target = calculateTargetForBalance();
        ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
        SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }



    private Transform3d calculateTargetForBalance() {
        currentPitch = SwerveSubsystem.gyro.getPitch();
        Transform3d target = new Transform3d();
        if(!reachedRamp && Math.abs(currentPitch) < 10) {
            target = new Transform3d(new Translation3d(0,-1.0,0), new Rotation3d());
        } else {
            reachedRamp = true;
        }
        if(reachedRamp && reachedLevel < Constants.Auton.balanceReverseDelay) {
            target = new Transform3d(new Translation3d(0,currentPitch/36,0), new Rotation3d()); 
            if(currentPitch >5) { //higher=stop earlier
                target = new Transform3d(new Translation3d(0,currentPitch/42,0), new Rotation3d(0,0,0));
                reachedLevel++;
            } 
        } else if(reachedRamp && reachedLevel < Constants.Auton.balanceReverseDelay + 6) { ///PLUS 5 ORIGINALLY
            target = new Transform3d(new Translation3d(), new Rotation3d(0,0,5));
            reachedLevel++;
        }

        return target;
    }

    public void gotoTag(int camNum, int tagNum) {
        Transform3d target = calculateTagPos(camNum,tagNum);
        ChassisSpeeds chassisSpeeds = calculateSpeedsToTarget(target);
        SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
        if(target.getY() > 1) { 
            swerveSubsystem.setModuleStates(swerveModuleStates);
        }
    }

    private Transform3d calculateTagPos(int camId, int tagId) {
        String tagNum = String.valueOf(tagId);
        String camNum = String.valueOf(camId);
        System.out.print(table.getSubTable("processed1").getSubTable("tag3").getEntry("tx").getString(""));
        double x = Double.valueOf(table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("tx").getString("0"));
        double y = Double.valueOf(table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("ty").getString("0"));
        double z = Double.valueOf(table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("tz").getString("0"));
        //double yaw = table.getSubTable("processed" + camNum).getSubTable("tag" + tagNum).getEntry("yaw").getDouble(0.0);
        double yaw = 0;
        z--;
        Transform3d target = new Transform3d(new Translation3d(z,y/300,x), new Rotation3d(0,0,yaw));
        System.out.println(target);
        return target;
    }

    private ChassisSpeeds calculateSpeedsToTarget(Transform3d target) {
        double angularDistance = target.getRotation().getZ();
        //double rotMultiplier = Math.abs(angularDistance / Constants.Swerve.kMaxAngularSpeedRadiansPerSecond);

        double xSpeed = Math.min(target.getX(), Constants.Swerve.kTeleDriveMaxSpeedMetersPerSecond);
        double ySpeed = Math.min(target.getY(), Constants.Swerve.kTeleDriveMaxSpeedMetersPerSecond);
        double turnSpeed = Math.min(angularDistance, Constants.Swerve.kTeleDriveMaxAngularSpeedRadiansPerSecond);
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