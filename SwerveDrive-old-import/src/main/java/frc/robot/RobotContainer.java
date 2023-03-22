package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutonMover;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    @Config
    public final SendableChooser<String> lightChooser = new SendableChooser<>();
    private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

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
        new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> SwerveSubsystem.zeroHeading()));
    }

    public void autonomousInit() {
        swerveSubsystem.setDefaultCommand(autonCmd);
    }

    public void teleopInit() {
        swerveSubsystem.setDefaultCommand(teleopCmd);
    }

    public Command getAutonomousCommand() {
        return new AutonMover(swerveSubsystem);

        /* 
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                SwerveConstants.kMaxSpeedMetersPerSecond,
                SwerveConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(SwerveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, 0),
                        new Translation2d(0, 0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(SwerveConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(SwerveConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                SwerveConstants.kPThetaController, 0, 0, SwerveConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                SwerveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));


        */
    }
}
