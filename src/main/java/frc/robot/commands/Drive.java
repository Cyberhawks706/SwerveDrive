package frc.robot.commands;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

import frc.robot.Components;
import frc.robot.Constants;
import frc.robot.IO;

import edu.wpi.first.wpilibj.command.Command;

public class Drive extends Command {
	

	public Drive() {
		requires(Components.chassis);
	}

	public double zero(double XboxValue, double DeadZone){
		double xboxValue = XboxValue;
		if(Math.abs(xboxValue)<DeadZone){
				xboxValue = 0;
			}

		return xboxValue;
	}

	public static double prevAngle = Math.PI;
	public static double prevAngleDiff = 0;

	public void execute(){
			 //DRIVE CODE!!
			
			//Adding DeadZone

			//Drive 
			double xboxX = IO.xboxDrive.getRightX();
			double xboxY = IO.xboxDrive.getRightY();

			

			double driveAngle = 0;
			double power = Math.sqrt(Math.pow(xboxX, 2) + Math.pow(xboxY, 2))/1;
		
			if(xboxX != 0){
				driveAngle = 2 * Math.atan2(xboxY, -xboxX);
			}
			double angleDiff = prevAngle - driveAngle;
			
			
			
			prevAngleDiff = angleDiff;
			driveAngle = Components.sparkWheelTurnBL.motorPos + angleDiff;
			System.out.println("driveAngle: "+ driveAngle + "\tprevAngle: "+ prevAngle + "\tangleDiff: "+ angleDiff);
			Components.sparkWheelTurnFR.setPos(driveAngle);
			Components.sparkWheelTurnFL.setPos(driveAngle);
			Components.sparkWheelTurnBR.setPos(driveAngle);
			Components.sparkWheelTurnBL.setPos(driveAngle);
prevAngle = driveAngle;
			// Components.sparkWheelFR.setPower(power);
			// Components.sparkWheelFL.setPower(power);
			// Components.sparkWheelBR.setPower(power);
			// Components.sparkWheelBL.setPower(power);
			
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
