package frc.robot.commands;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

import frc.robot.Components;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LimelightTrack extends CommandBase {

    public LimelightTrack(Subsystem limelight){
        addRequirements(limelight);
    }
	

	public void execute(){
			 //tracking code

            double x = Limelight.tx.getDouble(0.0);
            double leftPower;
            double rightPower;

            
            if(Math.abs(x)<0.1){         // Case 1: Already close to target
                leftPower = 0;
                rightPower = 0;
            } else if (x==0){            // Case 2: Target not in sight
                rightPower = -0.2;
                leftPower = 0.2;
            } else {                     // Case 3: Target in sight but not close
                rightPower = x/200;
                leftPower = -x/200;
            }



          

	}
    @Override
	public void initialize() {
	}
    @Override
	public boolean isFinished() {
		return false;
	}

	protected void end() {
	}

	protected void interrupted() {
	}
}
