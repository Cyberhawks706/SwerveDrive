package frc.robot.subsystems;




import frc.robot.Components;

import frc.robot.IO;
import frc.robot.Robot;
import frc.robot.commands.Drive;
import frc.robot.config.Config;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Chassis extends Subsystem {


	public static boolean piInControl = false;

	public void initDefaultCommand() {
		setDefaultCommand(new Drive());
		
	}

	int valuesAdded = 0;
	double gyroTotal = 0;
	int totalValues = 10;
	
	public double extendOffset = 0;
    public double rotationOffset = 0;

	public void periodic() {

		/* if(IO.xboxManipulator.getAButtonPressed()){
			Components.limelighttrack.execute();
		} else {
			Components.drive.execute();
		}
	
		/* 
		if(valuesAdded < totalValues){
			gyroTotal += (double) Components.ahrs.getYaw();
			valuesAdded++;
		}else{
			valuesAdded = 0;
			Config.gyroAngle.value = gyroTotal/ ((double) totalValues);
			gyroTotal = 0;
		}

		if(IO.xboxDrive.getAButtonPressed())Components.ahrs.calibrate();//calibrate when needed
		if(IO.xboxDrive.getYButtonPressed())Components.ahrs.zeroYaw();
		*/

		//Components.sparkWheelBR.setPower(0.25);
		//Components.sparkWheelBL.setPower(0.25);
		//Components.sparkWheelFR.setPower(0.25);
		//Components.sparkWheelFL.setPower(0.25);

		Components.sparkWheelTurnBR.setPos(4*Math.PI);
		Components.sparkWheelTurnBL.setPos(4*Math.PI);
		Components.sparkWheelTurnFR.setPos(4*Math.PI);
		Components.sparkWheelTurnFL.setPos(4*Math.PI);
	
		
	}
 
	public double previousAngle = 0;

	public void zeroAllMotors() {

	}
	public void Drive(double leftSpeed, double rightSpeed) {
		
	}

	public void ShootyShoot(double leftSpeed, double rightSpeed){

	}
}
