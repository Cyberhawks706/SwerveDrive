package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.CameraDaemon;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;


public class Robot extends TimedRobot {

	public static boolean emergencyDisabled = false;

	//Command autonomousCommand;
	//SendableChooser<Command> chooser = new SendableChooser<>();


	public static Chassis chassis;
	public static boolean auto;
	public static XboxController xboxDrive;
	public static SwerveModulePosition[] modulePosition;
	public static SwerveJoystickCmd swerveJoystickCmd;



	

	private RobotContainer m_robotContainer;
	

	@Override
	public void robotInit() {
		CameraDaemon.robotInit(); //Starts cameraserver
		System.out.println("Started");
		Components.init();
		m_robotContainer = new RobotContainer();


		//Components.ahrs.calibrate();
		
	}

	@Override
	public void robotPeriodic(){
		
		CommandScheduler.getInstance().run();

		
	}

	@Override
	public void disabledInit() {

	}


	@Override
	public void disabledPeriodic() {
	}


	@Override
	public void autonomousInit() {
		//m_autonomouscommand = m_robotContainer.getAutonomousCommand();
	}


	@Override
	public void autonomousPeriodic() {
		//Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
	
	}

	@Override
	public void teleopPeriodic() {

		

	}

}


