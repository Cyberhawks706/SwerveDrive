package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {


	private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	static final PowerDistribution m_pdp = new PowerDistribution(22, ModuleType.kRev);
	private static final GenericHID thrustJoystick = new GenericHID(1);
	private static final CommandXboxController driverJoystick = new CommandXboxController(2);
	public static final XboxController manipulatorJoystick = new XboxController(3);

	public RobotContainer() {
		swerveSubsystem.setDefaultCommand(swerveSubsystem.xboxDriveCommand(driverJoystick));
		if (thrustJoystick.isConnected()) {
		swerveSubsystem.setDefaultCommand(new RunCommand(() -> swerveSubsystem.teleDrive(
			-thrustJoystick.getRawAxis(1),
			-thrustJoystick.getRawAxis(0),
			-thrustJoystick.getRawAxis(2),
			Math.abs((thrustJoystick.getRawAxis(3)-1)/2)), swerveSubsystem));
		}
		DriverStation.silenceJoystickConnectionWarning(true);
		configureButtonBindings(); 
	}

	private void configureButtonBindings() {
		driverJoystick.a().onTrue(new InstantCommand(() -> swerveSubsystem.recenter()));
		new Trigger(()-> thrustJoystick.getRawButton(1)).onTrue(new InstantCommand(() -> swerveSubsystem.recenter()));
	}

	public Command getAutonomousCommand() {
		// return new AutonMover(swerveSubsystem);

		// 1. Create trajectory settings
		// TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
		// Constants.Swerve.kMaxSpeedMetersPerSecond,
		// Constants.Swerve.kMaxAccelerationMetersPerSecondSquared)
		// .setKinematics(Constants.Swerve.kDriveKinematics);

		// 2. Generate trajectory
		PathPlannerTrajectory traj = PathPlanner.loadPath("New New Path", 0.5, 0.35);
		swerveSubsystem.m_field.getObject("traj").setTrajectory(traj);

		return swerveSubsystem.followTrajectoryCommand(traj, true);
		// 3. Define PID controllers for tracking trajectory
		// PIDController xController = new PIDController(0, 0, 0);
		// PIDController yController = new PIDController(Constants.Swerve.kPYController, 0, 0);
		// ProfiledPIDController thetaController = new ProfiledPIDController(
		// 		Constants.Swerve.kPThetaController, 0, 0, Constants.Swerve.kThetaControllerConstraints);
		// thetaController.enableContinuousInput(-Math.PI, Math.PI);

		// // 4. Construct command to follow trajectory
		// PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
		// 		traj,
		// 		swerveSubsystem::getPose, // Pose supplier, swerveSubsystem::getPose
		// 		swerveSubsystem.getKinematics(), // SwerveDriveKinematics
		// 		xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
		// 		yController, // Y controller (usually the same values as X controller)
		// 		new PIDController(0.3, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
		// 		swerveSubsystem::setModuleStates, // Module states consumer
		// 		false, // Should the path be automatically mirrored depending on alliance color.
		// 				// Optional, defaults to true
		// 		swerveSubsystem // Requires this drive subsystem
		// );

		// // traj.getInitialHolonomicPose().getTranslation()
		// // 5. Add some init and wrap-up, and return everything
		// return new SequentialCommandGroup(
		// 		new InstantCommand(
		// 				() -> swerveSubsystem.resetOdometry(new Pose2d(-traj.getInitialHolonomicPose().getY() / 3,
		// 						traj.getInitialHolonomicPose().getX() / 3, Rotation2d.fromDegrees(-90)))), // new
		// 																									// Pose2d(-1,0.3,Rotation2d.fromDegrees(-90))
		// 		swerveControllerCommand,
		// 		new InstantCommand(() -> swerveSubsystem.stopModules()));

	}
}
