package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Vision extends SubsystemBase {

	public void initDefaultCommand() {
		 setDefaultCommand(new Limelight());
	}

	private void setDefaultCommand(Limelight limelight) {
	}

	public void periodic() {
	}

}