package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public final class Vision extends Subsystem {

	public void initDefaultCommand() {
		 setDefaultCommand(new Limelight());
	}

	private void setDefaultCommand(Limelight limelight) {
	}

	public void periodic() {
	}

}