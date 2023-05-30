package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BrushlessSparkWithPID;
import frc.robot.Constants;

import static frc.robot.Constants.Arm.*;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class ArmSubsystem extends SubsystemBase{
    private final BrushlessSparkWithPID sparkLiftF;
    private final BrushlessSparkWithPID sparkLiftR;
    private final BrushlessSparkWithPID sparkIntakeTilt;

    private final AnalogPotentiometer frontLiftPot;
    private final AnalogPotentiometer rearLiftPot;

    private double fSetpoint = 0;
    private double rSetpoint = 0;
    private double tiltSetpoint = 0;

    private double fSpeed = 0;
    private double rSpeed = 0;
    private double tiltSpeed = 0;

    public ArmSubsystem() {
        sparkLiftF = new BrushlessSparkWithPID(frontLiftMotorId, 0.0006, 0.0000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 4000, Constants.PID.Wheels.minVel, maxAcc,Constants.PID.Wheels.allowedErr);
        sparkLiftR = new BrushlessSparkWithPID(rearLiftMotorId, 0.0006, 0.000001, 0.001, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 4000, Constants.PID.Wheels.minVel, maxAcc, Constants.PID.Wheels.allowedErr);
        sparkIntakeTilt = new BrushlessSparkWithPID(4, 0.0005, 0.000002, 0, 0, 0, -0.5, 0.5, 2000, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, 0.03); //10^-21, 10^-7, 10^-35, 0.07
        sparkIntakeTilt.spark.getPIDController().setIMaxAccum(20, 0);
        sparkIntakeTilt.spark.setSoftLimit(SoftLimitDirection.kReverse, 0);
        frontLiftPot = new AnalogPotentiometer(frontLiftPotPort, 3.48, -0.2);
        rearLiftPot = new AnalogPotentiometer(rearLiftPotPort, 3.58,-0.3);
       
        fSetpoint = getFPot();
        rSetpoint = getRPot();
        tiltSetpoint = getTiltPos();
    }

    public double getFPot() {
        return frontLiftPot.get();
    }
    
    public double getRPot() {
        return rearLiftPot.get();
    }

    public double getTiltPos() {
        return sparkIntakeTilt.getPos();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double fDiff = MathUtil.clamp(fSetpoint - getFPot(), -0.5, 0.5);
        double rDiff = MathUtil.clamp(rSetpoint - getRPot(), -0.25, 0.25);
        double tiltDiff = MathUtil.clamp(tiltSetpoint - getTiltPos(), -0.25, 0.25);
        //System.out.println(tiltSetpoint + " " + getTiltPos());
        if(fSpeed == 0 && rSpeed == 0 && tiltSpeed == 0) {
            if(reachedSetpoint()) {
                // sparkLiftF.setPos(sparkLiftF.getPos());
                // sparkLiftR.setPos(sparkLiftR.getPos());
                sparkIntakeTilt.setPos(sparkIntakeTilt.getPos());
                //sparkIntakeTilt.setPower(tiltDiff/5);
            }
            else {
            //     sparkLiftF.setPower(fDiff);
            //     sparkLiftR.setPower(rDiff);
            //     sparkIntakeTilt.setPower(tiltDiff);
            // sparkLiftF.setPos(sparkLiftF.getPos());
            // sparkLiftR.setPos(sparkLiftR.getPos());
            sparkIntakeTilt.setPos(tiltSetpoint);
            }
        } else {
            // sparkLiftF.setPower(fSpeed);
            // sparkLiftR.setPower(rSpeed);
            sparkIntakeTilt.setPower(tiltSpeed);
        }
        
    }
    /**
     * Checks if all setpoints have been reached
     * @return true if all setpoints have been reached
     */
    public boolean reachedSetpoint() {
        return Math.abs(getFPot() - fSetpoint) < heightTolerance 
            && Math.abs(getRPot() - rSetpoint) < heightTolerance
            && Math.abs(getTiltPos() - tiltSetpoint) < tiltTolerance;
    }

    /**
     * Sets the setpoints for the arm
     * @param fSetpoint height of front lift
     * @param rSetpoint height of rear lift
     * @param tiltSetpoint intake tilt angle
     */
    public void setPositions(double fSetpoint, double rSetpoint, double tiltSetpoint) {
        this.fSetpoint = fSetpoint;
        this.rSetpoint = rSetpoint;
        this.tiltSetpoint = tiltSetpoint;
        fSpeed = 0;
        rSpeed = 0;
        tiltSpeed = 0;
    }
     /**
      * Sets the setpoints for the arm
      * @param setpoints array of setpoints, in order of front lift, rear lift, intake tilt
      */
    public void setPositions(double[] setpoints) {
        this.fSetpoint = setpoints[0];
        this.rSetpoint = setpoints[1];
        this.tiltSetpoint = setpoints[2];
        fSpeed = 0;
        rSpeed = 0;
        tiltSpeed = 0;
    }
    /**
     * Sets the speeds for the arm
     * @param fSpeed speed of front lift, positive is up
     * @param rSpeed speed of rear lift, positive is up
     * @param tiltSpeed speed of intake tilt
     */
    public void setSpeeds(double fSpeed, double rSpeed, double tiltSpeed) {
        this.fSpeed = fSpeed;
        this.rSpeed = rSpeed;
        this.tiltSpeed = tiltSpeed;
        fSetpoint = getFPot();
        rSetpoint = getRPot();
        tiltSetpoint = getTiltPos();
    }

    public Command setPositionsCommand(double[] setpoints) {
        return this.runOnce(() -> setPositions(setpoints));
    }

    public Command resetIntakeCommand() {
        return this.runOnce(() -> sparkIntakeTilt.rezero());
    }
}
