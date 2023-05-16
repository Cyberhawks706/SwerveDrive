package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MoveArm;
import frc.robot.commands.XboxDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {


	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final ArmSubsystem arm = new ArmSubsystem();
	private final Intake intake = new Intake();
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();
	private final CommandGenericHID thrustJoystick = new CommandGenericHID(1);
	private final CommandXboxController driverJoystick = new CommandXboxController(2);
	private final CommandXboxController manipulatorJoystick = new CommandXboxController(3);

	private Command armCommand = arm.runEnd(() -> arm.setSpeeds(
		manipulatorJoystick.getLeftY(), 
		manipulatorJoystick.getRightY(),
		manipulatorJoystick.getRightTriggerAxis() - manipulatorJoystick.getLeftTriggerAxis()), () -> arm.setSpeeds(0, 0, 0));

	public RobotContainer() {
		swerveSubsystem.setDefaultCommand(new XboxDriveCommand(driverJoystick, swerveSubsystem));
		// if (thrustJoystick.getHID().isConnected()) {
		// 	swerveSubsystem.setDefaultCommand(new RunCommand(() -> swerveSubsystem.calculate(
		// 		-thrustJoystick.getRawAxis(1),
		// 		-thrustJoystick.getRawAxis(0),
		// 		-thrustJoystick.getRawAxis(2),
		// 		Math.abs((thrustJoystick.getRawAxis(3)-1)/2)), swerveSubsystem));
		// }
		DriverStation.silenceJoystickConnectionWarning(true);
		configureButtonBindings(); 
		configureDashboard();
	}

	private void configureDashboard() {
		autoChooser.setDefaultOption("None", Commands.none());
		try (Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner"))) {
			files.filter(file -> !Files.isDirectory(file))
				.map(Path::getFileName)
				.map(Path::toString)
				.filter(fileName -> fileName.endsWith(".path"))
				.sorted()
				.map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
				.forEach(pathName -> autoChooser.addOption(pathName,
					swerveSubsystem.followTrajectoryCommand(PathPlanner.loadPath(pathName, 0.5, 0.35), true)));
		} catch (IOException e) {
			System.out.println("********* Failed to list PathPlanner paths. *********");
		}
		Shuffleboard.getTab("Espresso").add("Auto Chooser", autoChooser);
	}

	private void configureButtonBindings() {
		driverJoystick.a().onTrue(new InstantCommand(() -> swerveSubsystem.recenter()));
		thrustJoystick.button(1).onTrue(new InstantCommand(() -> swerveSubsystem.recenter()));

		manipulatorJoystick.leftStick().or(manipulatorJoystick.rightStick()).whileTrue(armCommand);
		manipulatorJoystick.rightBumper().whileTrue(Commands.startEnd(() -> intake.set(1), () -> intake.stop(), intake));
		manipulatorJoystick.leftBumper().whileTrue(Commands.startEnd(() -> intake.set(-1), () -> intake.stop(), intake));
		manipulatorJoystick.a().
			onTrue(new MoveArm(arm, Constants.ArmPositions.groundPickup).unless(
				manipulatorJoystick.rightBumper().or(manipulatorJoystick.leftBumper())
		));
		manipulatorJoystick.a().and(manipulatorJoystick.leftBumper()).onTrue(new MoveArm(arm, Constants.ArmPositions.cubePickup));
		manipulatorJoystick.a().and(manipulatorJoystick.rightBumper()).onTrue(new MoveArm(arm, Constants.ArmPositions.conePickup));
		manipulatorJoystick.y().onTrue(new MoveArm(arm, Constants.ArmPositions.humanPickup));
		manipulatorJoystick.x().onTrue(new MoveArm(arm, Constants.ArmPositions.topCone));
		manipulatorJoystick.b().onTrue(new MoveArm(arm, Constants.ArmPositions.midCone));
		manipulatorJoystick.povRight().onTrue(new MoveArm(arm, Constants.ArmPositions.midCube));
		manipulatorJoystick.povLeft().onTrue(new MoveArm(arm, Constants.ArmPositions.topCube));
		manipulatorJoystick.back().onTrue(new MoveArm(arm, Constants.ArmPositions.safeDrive));
	}

	public Command getAutonomousCommand() {
		// return new AutonMover(swerveSubsystem);
		return autoChooser.getSelected();
		// 2. Generate trajectory
		// PathPlannerTrajectory traj = PathPlanner.loadPath("New New Path", 0.5, 0.35);
		// swerveSubsystem.m_field.getObject("traj").setTrajectory(traj);

		// return swerveSubsystem.followTrajectoryCommand(traj, true);
	}
}
