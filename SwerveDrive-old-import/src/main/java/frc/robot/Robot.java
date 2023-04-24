package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutonMover;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Lighting;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	public static boolean emergencyDisabled = false;

	public static SwerveJoystickCmd swerveJoystickCmd;


	private RobotContainer m_robotContainer;
	

	@Override
	public void robotInit() {


	// 	    /$$$$$$  /$$       /$$             /$$      /$$ /$$   /$$ /$$$$$$$$ /$$$$$$$$ /$$        /$$$$$$                                     
	//     /$$__  $$| $$      | $$            | $$  /$ | $$| $$  | $$| $$_____/| $$_____/| $$       /$$__  $$                                    
	//    | $$  \ $$| $$      | $$            | $$ /$$$| $$| $$  | $$| $$      | $$      | $$      | $$  \__/                                    
	//    | $$$$$$$$| $$      | $$            | $$/$$ $$ $$| $$$$$$$$| $$$$$   | $$$$$   | $$      |  $$$$$$                                     
	//    | $$__  $$| $$      | $$            | $$$$_  $$$$| $$__  $$| $$__/   | $$__/   | $$       \____  $$                                    
	//    | $$  | $$| $$      | $$            | $$$/ \  $$$| $$  | $$| $$      | $$      | $$       /$$  \ $$                                    
	//    | $$  | $$| $$$$$$$$| $$$$$$$$      | $$/   \  $$| $$  | $$| $$$$$$$$| $$$$$$$$| $$$$$$$$|  $$$$$$/                                    
	//    |__/  |__/|________/|________/      |__/     \__/|__/  |__/|________/|________/|________/ \______/                                     
																																			  
																																			  
																																			  
	//     /$$      /$$ /$$   /$$  /$$$$$$  /$$$$$$$$       /$$$$$$$$ /$$$$$$   /$$$$$$  /$$$$$$$$       /$$$$$$$$ /$$   /$$ /$$$$$$$$           
	//    | $$$    /$$$| $$  | $$ /$$__  $$|__  $$__/      | $$_____//$$__  $$ /$$__  $$| $$_____/      |__  $$__/| $$  | $$| $$_____/           
	//    | $$$$  /$$$$| $$  | $$| $$  \__/   | $$         | $$     | $$  \ $$| $$  \__/| $$               | $$   | $$  | $$| $$                 
	//    | $$ $$/$$ $$| $$  | $$|  $$$$$$    | $$         | $$$$$  | $$$$$$$$| $$      | $$$$$            | $$   | $$$$$$$$| $$$$$              
	//    | $$  $$$| $$| $$  | $$ \____  $$   | $$         | $$__/  | $$__  $$| $$      | $$__/            | $$   | $$__  $$| $$__/              
	//    | $$\  $ | $$| $$  | $$ /$$  \ $$   | $$         | $$     | $$  | $$| $$    $$| $$               | $$   | $$  | $$| $$                 
	//    | $$ \/  | $$|  $$$$$$/|  $$$$$$/   | $$         | $$     | $$  | $$|  $$$$$$/| $$$$$$$$         | $$   | $$  | $$| $$$$$$$$           
	//    |__/     |__/ \______/  \______/    |__/         |__/     |__/  |__/ \______/ |________/         |__/   |__/  |__/|________/           
																																			  
																																			  
																																			  
	// 	 /$$$$$$   /$$$$$$  /$$      /$$ /$$$$$$$$       /$$$$$$$  /$$$$$$ /$$$$$$$  /$$$$$$$$  /$$$$$$  /$$$$$$$$ /$$$$$$  /$$$$$$  /$$   /$$
	// 	/$$__  $$ /$$__  $$| $$$    /$$$| $$_____/      | $$__  $$|_  $$_/| $$__  $$| $$_____/ /$$__  $$|__  $$__/|_  $$_/ /$$__  $$| $$$ | $$
	// | $$  \__/| $$  \ $$| $$$$  /$$$$| $$            | $$  \ $$  | $$  | $$  \ $$| $$      | $$  \__/   | $$     | $$  | $$  \ $$| $$$$| $$
	// |  $$$$$$ | $$$$$$$$| $$ $$/$$ $$| $$$$$         | $$  | $$  | $$  | $$$$$$$/| $$$$$   | $$         | $$     | $$  | $$  | $$| $$ $$ $$
	// 	\____  $$| $$__  $$| $$  $$$| $$| $$__/         | $$  | $$  | $$  | $$__  $$| $$__/   | $$         | $$     | $$  | $$  | $$| $$  $$$$
	// 	/$$  \ $$| $$  | $$| $$\  $ | $$| $$            | $$  | $$  | $$  | $$  \ $$| $$      | $$    $$   | $$     | $$  | $$  | $$| $$\  $$$
	// |  $$$$$$/| $$  | $$| $$ \/  | $$| $$$$$$$$      | $$$$$$$/ /$$$$$$| $$  | $$| $$$$$$$$|  $$$$$$/   | $$    /$$$$$$|  $$$$$$/| $$ \  $$
	// 	\______/ |__/  |__/|__/     |__/|________/      |_______/ |______/|__/  |__/|________/ \______/    |__/   |______/ \______/ |__/  \__/
																																			  
																							  
	
		System.out.println("Started");
		Components.init();
		m_robotContainer = new RobotContainer();
		Shuffleboard.getTab("Espresso").addBoolean("Intake Switch", () -> !Components.intakeSwitch.get());
		PathPlannerServer.startServer(5811); // 5811 = port number. adjust this according to your needs
		
		//Components.ahrs.calibrate();
		
	}
	@Override
	public void robotPeriodic(){
		CommandScheduler.getInstance().run();
		SmartDashboard.putData(RobotContainer.m_pdp);
		SmartDashboard.putNumber("Front", Components.frontLiftPot.get());
		SmartDashboard.putNumber("Rear", Components.rearLiftPot.get());
		Lighting.setLEDS(NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Espresso").getSubTable("Lighting").getEntry("Active").getString(""));
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
	}


	@Override
	public void disabledPeriodic() {
	}


	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
	  m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  
	  // schedule the autonomous command (example)
	  if (m_autonomousCommand != null) {
		m_autonomousCommand.schedule();
		AutonMover.init();
	  }
	}


	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {

		

	}

}


