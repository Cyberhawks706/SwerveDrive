package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A command that sets the arm to a given position, finishes when the arm reaches the setpoint.
 */
public class MoveArm extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double[] positions;

    /**
     * Creates a new SetArmPositions.
     * @param armSubsystem The subsystem used by this command.
     * @param positions The positions to set the arm to, in order of front, back, tilt.
     */
    public MoveArm(ArmSubsystem armSubsystem, double[] positions) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;
        this.positions = positions;
    }

    @Override
    public void initialize() {
        armSubsystem.setPositions(positions);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.reachedSetpoint();
    }
}
