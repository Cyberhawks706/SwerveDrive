package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.Drive;
import frc.robot.commands.LimelightTrack;
import frc.robot.subsystems.CameraDaemon;
import frc.robot.subsystems.Chassis;

import com.revrobotics.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
//color stuff
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class Robot extends TimedRobot {

	public static boolean emergencyDisabled = false;

	//Command autonomousCommand;
	//SendableChooser<Command> chooser = new SendableChooser<>();


	public static Chassis chassis;
	public static boolean auto;
	public static XboxController xboxDrive;
	

	



	public void robotInit() {
		CameraDaemon.robotInit(); //Starts cameraserver
		System.out.println("Started");
		Components.init();
		auto = DriverStation.isAutonomous();


	
		//Components.ahrs.calibrate();
		
	}
	public void disabledInit() {

	}

	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();
	}

	public void autonomousInit() {
		Auton.run();
	}

	public void autonomousPeriodic() {
		//Scheduler.getInstance().run();
	}

	public void teleopInit() {
	}

	public void teleopPeriodic() {
		Components.drive.execute();
	}

}


