package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    public static SwerveModuleState[] moduleStates;
    public static ChassisSpeeds chassisSpeeds;

    public static boolean OverrideClaw = false;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs

        double rSpeed = 0;
        double fSpeed = 0;
        if (RobotContainer.manipulatorJoystick.getRawButton(10)) {
            rSpeed = RobotContainer.manipulatorJoystick.getRightY();
        }
        if (RobotContainer.manipulatorJoystick.getRawButton(9)) {
            fSpeed = RobotContainer.manipulatorJoystick.getLeftY();
        }
                

        double fLiftMotorPos = Components.sparkLiftF.encoder.getPosition();
        double rLiftMotorPos = Components.sparkLiftR.encoder.getPosition();
        
        // 2. Apply deadband
        
        fSpeed = Math.abs(fSpeed) > 0.1 ? fSpeed : 0.0;
        rSpeed = Math.abs(rSpeed) > 0.1 ? rSpeed : 0.0;
        // 3. Make the driving smoother
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

        if (RobotContainer.manipulatorJoystick.getAButton() && RobotContainer.manipulatorJoystick.getLeftBumper()) { // Ground Pickup Position
            desiredClawPos = 11.2;// 12.619
            desiredRLift = 2.77;// 2.919
            desiredFLift = 0.87;

            fSpeed = 1 * (frontPotPos - desiredFLift); // front number changes deceleration rate (Higher is quicker)
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
        else if (Math.abs(rSpeed) < 0.025)
            rSpeed /= 20;
        else if (Math.abs(rSpeed) < 0.1)
            rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
        else if (Math.abs(rSpeed) < 0.4)
            rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));
            

            clawTiltPos = desiredClawPos;

        }

         else if (RobotContainer.manipulatorJoystick.getAButton() && RobotContainer.manipulatorJoystick.getRightBumper()) { // Ground Pickup Position
            desiredClawPos = 15.4;// 12.619
            desiredRLift = 2.77;// 2.919
            desiredFLift = 1.81;

            fSpeed = 1 * (frontPotPos - desiredFLift); // front number changes deceleration rate (Higher is quicker)
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
        else if (Math.abs(rSpeed) < 0.025)
            rSpeed /= 20;
        else if (Math.abs(rSpeed) < 0.1)
            rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
        else if (Math.abs(rSpeed) < 0.4)
            rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));

            clawTiltPos = desiredClawPos;

        }
        
        else if (RobotContainer.manipulatorJoystick.getAButton()) { // Ground Pickup Position
            desiredClawPos = 13.5;// 12.619
            desiredRLift = 2.77;// 2.919
            desiredFLift = 1.4;

            fSpeed = 1 * (frontPotPos - desiredFLift); // front number changes deceleration rate (Higher is quicker)
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
        else if (Math.abs(rSpeed) < 0.025)
            rSpeed /= 20;
        else if (Math.abs(rSpeed) < 0.1)
            rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
        else if (Math.abs(rSpeed) < 0.4)
            rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));

            clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getYButton()) { // Human Pickup Position
            desiredClawPos = 19.5;
            desiredRLift = 2.756;
            desiredFLift = 2.865;

            fSpeed = 1.5 * (frontPotPos - desiredFLift);
            rSpeed = 1.5 * (rearPotPos - desiredRLift);

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

            clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getRawButton(7)) { // Safe Drive Position BACK BUTTON
            desiredClawPos = 3;
            desiredRLift = 3.1;
            desiredFLift = 1;

            fSpeed = 1 * (frontPotPos - desiredFLift);
            rSpeed = 1 * (rearPotPos - desiredRLift);

            if(Components.sparkClawTilt.motorPos<6){
            if (Math.abs(fSpeed) > 1) // Slow Down/Speed Up Front Arm
                fSpeed = fSpeed / Math.abs(fSpeed);
            else if (Math.abs(fSpeed) < 0.025)
                fSpeed /= 20;
            else if (Math.abs(fSpeed) < 0.1)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.4)
                fSpeed = 0.4 * (fSpeed / Math.abs(fSpeed));
            }

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

        }

        else if (RobotContainer.manipulatorJoystick.getXButton()) { // Top Cone Score
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
            desiredClawPos = 16.7;// 18.3
            desiredRLift = 2.71;// 2.69
            desiredFLift = 2.59;// 2.528

            fSpeed = 1.5 * (frontPotPos - desiredFLift);

            if (frontPotPos > 1.5)
                rSpeed = 1.5 * (rearPotPos - desiredRLift);

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
            else if (Math.abs(rSpeed) < 0.025)
                rSpeed /= 20;
            else if (Math.abs(rSpeed) < 0.1)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.4)
                rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));

            if (frontPotPos > 2)
                clawTiltPos = desiredClawPos;

        }

        else if (RobotContainer.manipulatorJoystick.getPOV() == 90) { // Mid Cube Score
            desiredClawPos = 12;// 18.3
            desiredRLift = 2.69;// 2.69
            desiredFLift = 2.42;// 2.528

            fSpeed = 1.5 * (frontPotPos - desiredFLift);

            if (frontPotPos > 1.5)
                rSpeed = 1.5 * (rearPotPos - desiredRLift);

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
            else if (Math.abs(rSpeed) < 0.025)
                rSpeed /= 20;
            else if (Math.abs(rSpeed) < 0.1)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.4)
                rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));

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
            else if (Math.abs(fSpeed) < 0.025)
                fSpeed /= 20;
            else if (Math.abs(fSpeed) < 0.1)
                fSpeed = 0.2 * (fSpeed / Math.abs(fSpeed));
            else if (Math.abs(fSpeed) < 0.4)
                fSpeed = 0.4 * (fSpeed / Math.abs(fSpeed));

            if (Math.abs(rSpeed) > 1) // Slow Down/Speed up Rear Arm
                rSpeed = rSpeed / Math.abs(rSpeed);
            else if (Math.abs(rSpeed) < 0.025)
                rSpeed /= 20;
            else if (Math.abs(rSpeed) < 0.1)
                rSpeed = 0.2 * (rSpeed / Math.abs(rSpeed));
            else if (Math.abs(rSpeed) < 0.4)
                rSpeed = 0.4 * (rSpeed / Math.abs(rSpeed));

            if (frontPotPos > 2.45)
                clawTiltPos = desiredClawPos;

        }
        
        if (RobotContainer.manipulatorJoystick.getRawButton(10)) {
            rLiftMotorPos -= rSpeed * 0.25;
        }

        clawTiltPos += clawTiltSpeed * 6;


        double intakeSpeed = 0;
        if (RobotContainer.manipulatorJoystick.getRightBumper())
            intakeSpeed = 1;
        if (RobotContainer.manipulatorJoystick.getLeftBumper())
            intakeSpeed = -1;

        Components.sparkClawTilt.setPos(clawTiltPos);
        Components.sparkIntake.setPower(intakeSpeed);

        if (Math.abs(fSpeed) < 0.1)
            Components.sparkLiftF.setPos(fLiftMotorPos);
        else
            Components.sparkLiftF.setPower(-fSpeed);
        if (Math.abs(rSpeed) < 0.1)
            Components.sparkLiftR.setPos(rLiftMotorPos);
        else
            Components.sparkLiftR.setPower(-rSpeed * 0.5);

    //System.out.println("F" + frontPotPos);
    //System.out.println("F" + fSpeed);
    //System.out.println("RRR" + rearPotPos);
    //System.out.println(clawTiltPos);
    

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