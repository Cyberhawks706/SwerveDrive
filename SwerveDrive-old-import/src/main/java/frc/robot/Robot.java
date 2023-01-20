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


	public final static SwerveModule frontLeft = new SwerveModule(
		SwerveConstants.kFrontLeftDriveMotorPort,
		SwerveConstants.kFrontLeftTurningMotorPort,
		SwerveConstants.kFrontLeftDriveEncoderReversed,
		SwerveConstants.kFrontLeftTurningEncoderReversed);

	public final static SwerveModule frontRight = new SwerveModule(
		SwerveConstants.kFrontRightDriveMotorPort,
		SwerveConstants.kFrontRightTurningMotorPort,
		SwerveConstants.kFrontRightDriveEncoderReversed,
		SwerveConstants.kFrontRightTurningEncoderReversed);

	public final static SwerveModule backLeft = new SwerveModule(
		SwerveConstants.kBackLeftDriveMotorPort,
		SwerveConstants.kBackLeftTurningMotorPort,
		SwerveConstants.kBackLeftDriveEncoderReversed,
		SwerveConstants.kBackLeftTurningEncoderReversed);

	public final static SwerveModule backRight = new SwerveModule(
		SwerveConstants.kBackRightDriveMotorPort,
		SwerveConstants.kBackRightTurningMotorPort,
		SwerveConstants.kBackRightDriveEncoderReversed,
		SwerveConstants.kBackRightTurningEncoderReversed);

	

	private Command m_autonomouscommand;
	private RobotContainer m_robotContainer;
	

	@Override
	public void robotInit() {
		CameraDaemon.robotInit(); //Starts cameraserver
		System.out.println("Started");
		Components.init();
		auto = DriverStation.isAutonomous();
		m_robotContainer = new RobotContainer();
		swerveJoystickCmd = new SwerveJoystickCmd(null, null, null, null, null);

		
		
		modulePosition = new SwerveModulePosition[4];

		modulePosition[0] = new SwerveModulePosition(frontLeft.driveEncoder.getPosition(), frontRight.getState().angle);
		modulePosition[1] = new SwerveModulePosition(frontRight.driveEncoder.getPosition(), frontRight.getState().angle);
		modulePosition[2] = new SwerveModulePosition(backLeft.driveEncoder.getPosition(), frontRight.getState().angle);
		modulePosition[3] = new SwerveModulePosition(backRight.driveEncoder.getPosition(), frontRight.getState().angle);

		//Components.ahrs.calibrate();
		
	}

	@Override
	public void robotPeriodic(){
		
		//CommandScheduler.getInstance().run();

		swerveJoystickCmd.execute();

		

	

		
	}

	@Override
	public void disabledInit() {

	}


	@Override
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();
	}


	@Override
	public void autonomousInit() {
		m_autonomouscommand = m_robotContainer.getAutonomousCommand();
	}


	@Override
	public void autonomousPeriodic() {
		//Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		
		if(m_autonomouscommand != null){
			m_autonomouscommand.cancel();
		}
		


	}

	@Override
	public void teleopPeriodic() {

		

	}

}


