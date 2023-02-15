package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.config.Config;
import frc.robot.subsystems.CameraDaemon;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;


public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
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
																																			  
																							  
		CameraDaemon.robotInit(); //Starts cameraserver
		System.out.println("Started");
		Components.init();
		m_robotContainer = new RobotContainer();


		//Components.ahrs.calibrate();
		
	}

	@Override
	public void robotPeriodic(){
		
		CommandScheduler.getInstance().run();

		Config.inputXvalue.value = SwerveJoystickCmd.xSpeed;
        Config.inputXvalue.push();


			Config.outputSpeed.value = SwerveSubsystem.x;
			Config.outputSpeed.push();
		
		Components.frontLiftMotor.setPower(0.3);
		//Components.backLiftMotor.setPower(-0.3);


		

		
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


