package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements AutoCloseable{
    
    //private BrushlessSparkWithPID sparkIntake;
    private CANSparkMax sparkIntake; 

    public Intake() {
        //sparkIntake = new BrushlessSparkWithPID(3, 0.000006, 0.000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 11000, Constants.PID.Wheels.minVel, 2000, Constants.PID.Wheels.allowedErr);
        sparkIntake = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
        sparkIntake.setSmartCurrentLimit(90, 5);
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
