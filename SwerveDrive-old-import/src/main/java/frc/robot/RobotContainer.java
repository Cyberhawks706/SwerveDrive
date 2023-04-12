package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutonMover;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    public final SendableChooser<String> lightChooser = new SendableChooser<>();
    final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final SwerveJoystickCmd teleopCmd = new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoystick.getLeftY(),
        () -> driverJoystick.getLeftX(),
        () -> driverJoystick.getRightX(),
        () -> !driverJoystick.getAButtonPressed());

    private final AutonMover autonCmd = new AutonMover(swerveSubsystem);

    public static final PowerDistribution m_pdp = new PowerDistribution(22, ModuleType.kRev);

    public static final XboxController driverJoystick = new XboxController(2);
    public static final XboxController manipulatorJoystick = new XboxController(3);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getLeftY(),
                () -> driverJoystick.getLeftX(),
                () -> driverJoystick.getRightX(),
                () -> !driverJoystick.getAButtonPressed()));

        

                

        configureButtonBindings();

        lightChooser.setDefaultOption("Pink", "PINK");
        lightChooser.addOption("Black", "BLACK");
        lightChooser.addOption("Yellow", "YELLOW");
        lightChooser.addOption("Purple", "PURPLE");
        lightChooser.addOption("Red", "RED");
        lightChooser.addOption("Blue", "BLUE");
        Shuffleboard.getTab("Espresso").add("Lighting", lightChooser);
        
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> SwerveSubsystem.zeroHeading()).alongWith(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d()))));
        //new Trigger(() -> manipulatorJoystick.getPOV() == 270 ? true : false).debounce(1).onTrue(new InstantCommand(() -> Components.sparkClawTilt.rezero()));
    }

    public void autonomousInit() {
        swerveSubsystem.setDefaultCommand(autonCmd);
    }

    public void teleopInit() {
        swerveSubsystem.setDefaultCommand(teleopCmd);
    }

    public Command getAutonomousCommand() {
        //return new AutonMover(swerveSubsystem);

        
        // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         SwerveConstants.kMaxSpeedMetersPerSecond,
        //         SwerveConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(SwerveConstants.kDriveKinematics);

        // 2. Generate trajectory
        PathPlannerTrajectory traj = PathPlanner.loadPath("New Path", new PathConstraints(1, 0.5));
        swerveSubsystem.m_field.getObject("traj").setTrajectory(traj);
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(0, 0, 0);
        PIDController yController = new PIDController(SwerveConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                SwerveConstants.kPThetaController, 0, 0, SwerveConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
                traj, 
                swerveSubsystem::getPose, // Pose supplier
                SwerveConstants.kDriveKinematics, // SwerveDriveKinematics
                xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                xController, // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                swerveSubsystem::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerveSubsystem // Requires this drive subsystem
            );
        
        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(-90)))),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));


        
    }
}
