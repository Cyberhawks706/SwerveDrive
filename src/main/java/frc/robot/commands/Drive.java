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
	double y = 0;

	double FWD = -IO.xboxDrive.getLeftY();
	double STR = IO.xboxDrive.getLeftX();
	double RCW = IO.xboxDrive.getRightX();


	

	public Drive() {
    	requires(Components.chassis);
	}
    
	public void execute(){
     	/*double theta = Components.ahrs.getYaw()*Math.PI/180;
		double temp=FWD*Math.cos(theta)+STR*Math.sin(theta);
		STR=-FWD*Math.sin(theta)+STR*Math.cos(theta);
		FWD=temp;

		final double S=24;
		final double R=Math.sqrt(576+576);

		double A=STR-RCW*(S/R);
		double B=STR+RCW*(S/R);
		double C=STR-RCW*(S/R);
		double D=STR+RCW*(S/R);

		double ws1=Math.sqrt(B*B+C*C);
		double ws2=Math.sqrt(B*B+D*D);
		double ws3=Math.sqrt(A*A+D*D);
		double ws4=Math.sqrt(A*A+C*C);

		double wa1=((Math.atan2(B,C)*180/Math.PI)+180)/28.125;
		double wa2=((Math.atan2(B,D)*180/Math.PI)+180)/28.125;
		double wa3=((Math.atan2(A,D)*180/Math.PI)+180)/28.125;
		double wa4=((Math.atan2(A,C)*180/Math.PI)+180)/28.125;


		double max = ws1;
		if(ws2>max){
			max = ws2;
		}
		if(ws3>max){
			max = ws3;
		}
		if(ws4>max){
			max = ws4;
		}
		if(max>1){
			ws1/=max;
			ws2/=max;
			ws3/=max;
			ws4/=max;
		}


		Components.sparkWheelFR.setPos(wa1);
		Components.sparkWheelFL.setPos(wa2);
		Components.sparkWheelBL.setPos(wa3);
		Components.sparkWheelBR.setPos(wa4);

		Components.sparkWheelFR.setPos(ws1);
		Components.sparkWheelFR.setPos(ws2);
		Components.sparkWheelFR.setPos(ws3);
		Components.sparkWheelFR.setPos(ws4);
		*/
		

			
			//DRIVE CODE!!
			//Get stick axes
			double turnX = IO.xboxDrive.getRightX();
			double driveX = IO.xboxDrive.getLeftX();
			double driveY = IO.xboxDrive.getLeftY();

			//Angle of xbox right joystick, in radians ranging from -pi to pi
			double stickAngle = Math.atan2(driveY, -driveX);
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
			if(Math.abs(turnX) > 0.02) {
				turnOffset += turnX * 0.06;
			}

        	// Check if the joystick is in the deadzone
        	if(Math.abs(driveX) < 0.1 && Math.abs(driveY) < 0.1 && Math.abs(turnX) < 0.02){
            	//If the joystick is in the deadzone, set the angle to the previous angle
            	driveAngle = prevDriveAngle;
        	} else if (Math.abs(driveX) < 0.1 && Math.abs(driveY) < 0.1 && Math.abs(turnX) >= 0.02) {
				driveAngle = prevStickAngle + fullRotations*Constants.Drive.ROT_SIZE + turnOffset;
			} else {
            	//If the joystick is not in the deadzone, set the drive angle to
            	//the stick angle plus the number of full rotations times the size of a full rotation
				// plus the offset from previous turns
            	driveAngle = stickAngle + fullRotations*Constants.Drive.ROT_SIZE + turnOffset;
				power = Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2));
        	}
			
        	//move(driveAngle, power*Constants.Drive.PWR_MODIFIER, power*Constants.Drive.PWR_MODIFIER);
			//power=0;
			move(driveAngle, power, power, driveAngle%Constants.Drive.ROT_SIZE, turnX * 0.35);
			
			//System.out.println(driveAngle%Constants.Drive.ROT_SIZE);
			//System.out.println(driveAngle);
        	// Reset the previous angle to the current angle for the next iteration
        	prevStickAngle = stickAngle;
			prevDriveAngle = driveAngle;


			if(IO.xboxDrive.getAButtonPressed()){
				Components.sparkWheelTurnBL.rezero(driveAngle%Constants.Drive.ROT_SIZE);
				Components.sparkWheelTurnBR.rezero(driveAngle%Constants.Drive.ROT_SIZE);
				Components.sparkWheelTurnFL.rezero(driveAngle%Constants.Drive.ROT_SIZE);
				Components.sparkWheelTurnFR.rezero(driveAngle%Constants.Drive.ROT_SIZE);
				driveAngle = 0;
				turnOffset = 0;
				fullRotations = 0;
			}

			
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

