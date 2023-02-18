package frc.robot.subsystems;




import frc.robot.Components;

import frc.robot.IO;
import frc.robot.Robot;

import frc.robot.config.Config;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Components;


public class Chassis extends SubsystemBase {


	public static boolean piInControl = false;

	public void initDefaultCommand() {

		
	}

	int valuesAdded = 0;
	double gyroTotal = 0;
	int totalValues = 10;
	
	public double extendOffset = 0;
    public double rotationOffset = 0;

	public void periodic() {
		//double x = Components.ahrs.getYaw();

	}
 
	public double previousAngle = 0;

	public void zeroAllMotors() {

	}
	
}
