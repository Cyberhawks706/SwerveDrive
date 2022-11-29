package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Components;

// All cower in fear before the dashboard demon

public final class PIDDaemon extends Subsystem {

	public void initDefaultCommand() {
        //No default command, is a daemon
	}

	public void periodic() {
        //put udating PID constant code here\
        Components.sparkWheelFL.UpdateSensorValues();   
        Components.sparkWheelFR.UpdateSensorValues();   
        Components.sparkWheelBR.UpdateSensorValues();   
        Components.sparkWheelBL.UpdateSensorValues();   
        Components.sparkWheelTurnFL.UpdateSensorValues();   
        Components.sparkWheelTurnFR.UpdateSensorValues();   
        Components.sparkWheelTurnBR.UpdateSensorValues();   
        Components.sparkWheelTurnBL.UpdateSensorValues();  

        
        //Components.sparkIntakeLift.UpdateSensorValues();
        //Components.sparkIntakeRollers.UpdateSensorValues();   

    }

}