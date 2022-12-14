package frc.robot.commands;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

import frc.robot.Components;
import frc.robot.Constants;
import frc.robot.IO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;;

public class Drive extends Command {

	public double PI = Constants.Drive.PI;
	// Previous angle of the joystick, used to calculate the angle change
	public double prevStickAngle = PI;
	// Number of full rotations of the joystick since the last time the angle was reset
	public double fullRotations = 0;
	public double prevDriveAngle = 0;
	public double turnOffset = 0;
	double x = 0 ;
	double y = 0;

	public Drive() {
    	requires(Components.chassis);
	}
    
	public void execute(){
         	//DRIVE CODE!!
			//Get stick axes
			double xboxLX = IO.xboxDrive.getLeftX();
			double xboxRX = IO.xboxDrive.getRightX();
			double xboxRY = IO.xboxDrive.getRightY();

			//Angle of xbox right joystick, in radians ranging from -pi to pi
			double stickAngle = Math.atan2(xboxRY, -xboxRX);
			stickAngle = (PI/Math.PI)*stickAngle;
			//Add pi to get a range of 0 to 2pi for simplicity, then multiply by 2 to match motors
			stickAngle = 2*(stickAngle + 0.5*PI);
        	double driveAngle = 0; //Final angle of the motors, in radians
        	double power = 0;//Final power of the motors, ranging from 0 to 1
        	double angleDiff = prevStickAngle - stickAngle;//Difference between current stick angle and previous angle

        	//If the difference is greater than half a rotation, change the number of full rotations to reverse the direction of rotation
        	if(Math.abs(angleDiff) > (Constants.Drive.ROT_SIZE / 2)){
            	if(angleDiff > 0){
                	// If the difference is positive, the joystick was rotated clockwise, so the number of full rotations should be increased
                	fullRotations++;
            	} else {
                	// If the difference is negative, the joystick was rotated counterclockwise, so the number of full rotations should be decreased
                	fullRotations--;
            	}
        	}
			
			// Check if we need to change turnOffset
			if(Math.abs(xboxLX) > 0.1) {
				turnOffset += xboxLX * 0.1; //TODO: mess with turn speeds
			}

        	// Check if the joystick is in the deadzone
        	if(Math.abs(xboxRX) < 0.1 && Math.abs(xboxRY) < 0.1){
            	//If the joystick is in the deadzone, set the angle to the previous angle
            	driveAngle = prevDriveAngle;
        	} else {
            	//If the joystick is not in the deadzone, set the drive angle to
            	//the stick angle plus the number of full rotations times the size of a full rotation
				// plus the offset from previous turns
            	driveAngle = stickAngle + fullRotations*Constants.Drive.ROT_SIZE + turnOffset;
				power = Math.sqrt(Math.pow(xboxRX, 2) + Math.pow(xboxRY, 2));
        	}
			
        	//move(driveAngle, power*Constants.Drive.PWR_MODIFIER, power*Constants.Drive.PWR_MODIFIER);
			
			move(driveAngle, power, power, stickAngle, xboxLX * 0.35);
			
			System.out.println(stickAngle);
        	// Reset the previous angle to the current angle for the next iteration
        	prevStickAngle = stickAngle;
			prevDriveAngle = driveAngle;
	}


	/*
	* Moves the robot in the given direction at the given power
	* @param angle The angle of the direction to move in, in radians
	* @param power The power to move at, ranging from 0 to 1
	*/
	private void move(double angle, double leftPower, double rightPower, double stickAngle, double turnSpeed) {
    	// Set turn motors to drive angle
    	Components.sparkWheelTurnFR.setPos(angle);
    	Components.sparkWheelTurnFL.setPos(angle);
    	Components.sparkWheelTurnBR.setPos(angle);
    	Components.sparkWheelTurnBL.setPos(angle);
		leftPower *= Constants.Drive.PWR_MODIFIER;
		rightPower *= Constants.Drive.PWR_MODIFIER;
    	turnSpeed = -turnSpeed;
		if (stickAngle >= -PI/2 && stickAngle <= PI/2) { //forward
			Components.sparkWheelFR.setPower(rightPower+turnSpeed);
			Components.sparkWheelBR.setPower(rightPower+turnSpeed);
			Components.sparkWheelFL.setPower(leftPower-turnSpeed);
			Components.sparkWheelBL.setPower(leftPower-turnSpeed);
		} else if (stickAngle >= PI/2 && stickAngle <= 3*PI/2){ //left
			Components.sparkWheelFR.setPower(rightPower+turnSpeed);
			Components.sparkWheelFL.setPower(leftPower+turnSpeed);
			Components.sparkWheelBR.setPower(rightPower-turnSpeed);
			Components.sparkWheelBL.setPower(leftPower-turnSpeed);
		} else if (stickAngle >= 3*PI/2 && stickAngle <= 5*PI/2){ //back
			Components.sparkWheelFL.setPower(rightPower+turnSpeed);
			Components.sparkWheelBL.setPower(rightPower+turnSpeed);
			Components.sparkWheelFR.setPower(leftPower-turnSpeed);
			Components.sparkWheelBR.setPower(leftPower-turnSpeed);
		} else if (stickAngle >= 5*PI/2 || stickAngle <= -PI/2){ //right
			Components.sparkWheelBR.setPower(rightPower+turnSpeed);
			Components.sparkWheelBL.setPower(leftPower+turnSpeed);
			Components.sparkWheelFR.setPower(rightPower-turnSpeed);
			Components.sparkWheelFL.setPower(leftPower-turnSpeed);
		}
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

