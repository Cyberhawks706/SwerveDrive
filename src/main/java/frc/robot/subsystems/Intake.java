package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BrushlessSparkWithPID;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements AutoCloseable{
    
   // private BrushlessSparkWithPID sparkIntake;
    private CANSparkMax sparkIntake; 

    public Intake() {
        sparkIntake = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
        sparkIntake.setSmartCurrentLimit(95, 15);
        sparkIntake.setIdleMode(IdleMode.kBrake);
    }
    /*
     * @param speed the speed to set the intake to, 1 = cone IN, cube OUT
     */
    public void set(double speed) {
        sparkIntake.set(speed);
    }

    public void stop() {
        sparkIntake.stopMotor();
    }

    @Override
    public void close() throws Exception{
        sparkIntake.close();
    }

    public CANSparkMax getSpark() {
        return sparkIntake;
    }
}
