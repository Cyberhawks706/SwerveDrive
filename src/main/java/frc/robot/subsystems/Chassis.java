package frc.robot.subsystems;




import frc.robot.Components;

import frc.robot.IO;
import frc.robot.Robot;
import frc.robot.commands.Drive;
import frc.robot.config.Config;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


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
