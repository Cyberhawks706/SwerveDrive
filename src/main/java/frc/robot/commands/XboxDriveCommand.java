package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.*;

public class XboxDriveCommand extends CommandBase{
	private final CommandXboxController controller;
	private final SwerveSubsystem swerveSubsystem;
	
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

	public XboxDriveCommand(CommandXboxController controller, SwerveSubsystem swerveSubsystem) {
        this.controller = controller;
		this.swerveSubsystem = swerveSubsystem;
		
        this.xLimiter = new SlewRateLimiter(kMaxAccelTele);
        this.yLimiter = new SlewRateLimiter(kMaxAccelTele);
        this.turnLimiter = new SlewRateLimiter(kMaxAngularAccelTele);
    }

	@Override
	public void execute() {
		double x = -controller.getLeftX();
		double y = -controller.getLeftY();
		double rot = -controller.getRightX();
		double accelMultiplier = controller.getRightTriggerAxis();
		x = MathUtil.applyDeadband(x, Constants.IO.kDeadband);
        y = MathUtil.applyDeadband(y, Constants.IO.kDeadband);
        rot = MathUtil.applyDeadband(rot, Constants.IO.kDeadband);
		x = Math.copySign(x * x, x);
		y = Math.copySign(y * y, y);
		rot = Math.copySign(rot * rot, rot);
		x *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		y *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		rot *= MathUtil.interpolate(0.25, 1, accelMultiplier);
        x = xLimiter.calculate(x * kMaxVelTele);
        y = yLimiter.calculate(y * kMaxVelTele);
        rot = turnLimiter.calculate(rot * kMaxAngularVelTele);
		swerveSubsystem.drive(x, y, rot, true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}