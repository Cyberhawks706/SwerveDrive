package frc.robot.commands;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

import frc.robot.Components;
import frc.robot.Constants;
import frc.robot.IO;

import edu.wpi.first.wpilibj.command.Command;

public class Drive extends Command {

	// Previous angle of the joystick, used to calculate the angle change
	public double prevStickAngle = Math.PI;
	// Number of full rotations of the joystick since the last time the angle was reset
	public double fullRotations = 0;

	public Drive() {
    	requires(Components.chassis);
	}
    
	public void execute(){
         	//DRIVE CODE!!
			//Get stick axes
			double xboxX = IO.xboxDrive.getRightX();
			double xboxY = IO.xboxDrive.getRightY();

			//Angle of xbox right joystick, in radians ranging from -pi to pi
			double stickAngle = Math.atan2(xboxY, -xboxX);
			//Add pi to get a range of 0 to 2pi for simplicity, then multiply by 2 to match motors
			stickAngle = 2*(stickAngle + Math.PI);
        	double driveAngle = 0; //Final angle of the motors, in radians
        	double power = Math.sqrt(Math.pow(xboxX, 2) + Math.pow(xboxY, 2));//Final power of the motors, ranging from 0 to 1
        	double angleDiff = prevStickAngle - stickAngle;//Difference between current stick angle and previous angle

        	//If the difference is greater than half a rotation, change the number of full rotations to reverse the direction of rotation
        	if(Math.abs(angleDiff) > (Constants.Chassis.ROT_SIZE / 2)){
            	if(angleDiff > 0){
                	// If the difference is positive, the joystick was rotated clockwise, so the number of full rotations should be increased
                	fullRotations++;
            	} else {
                	// If the difference is negative, the joystick was rotated counterclockwise, so the number of full rotations should be decreased
                	fullRotations--;
            	}
        	}

        	// Check if the joystick is in the deadzone
        	if(Math.abs(xboxX) < 0.05 && Math.abs(xboxY) < 0.05){
            	//If the joystick is in the deadzone, set the angle to the previous angle
            	driveAngle = prevStickAngle;
        	} else {
            	//If the joystick is not in the deadzone, set the drive angle to
            	//the stick angle plus the number of full rotations times the size of a full rotation
            	driveAngle = stickAngle + fullRotations*Constants.Chassis.ROT_SIZE;
        	}

        	move(driveAngle, 0); //power set to 0 for testing
       	 
        	// Reset the previous angle to the current angle for the next iteration
        	prevStickAngle = stickAngle;
	}

	/*
	* Moves the robot in the given direction at the given power
	* @param angle The angle of the direction to move in, in radians
	* @param power The power to move at, ranging from 0 to 1
	*/
	private void move(double angle, double power) {
    	// Set turn motors to drive angle
    	Components.sparkWheelTurnFR.setPos(angle);
    	Components.sparkWheelTurnFL.setPos(angle);
    	Components.sparkWheelTurnBR.setPos(angle);
    	Components.sparkWheelTurnBL.setPos(angle);
   	 
    	// Set drive motors to drive power
    	Components.sparkWheelFR.setPower(power);
    	Components.sparkWheelFL.setPower(power);
    	Components.sparkWheelBR.setPower(power);
    	Components.sparkWheelBL.setPower(power);
	}

	protected void initialize() {
	}

	protected boolean isFinished() {
    	return false;
	}

	protected void end() {
	}

	protected void interrupted() {
	}
}

