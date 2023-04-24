package frc.robot;



//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;



public class Components {

    public static BrushlessSparkWithPID sparkLiftF;
    public static BrushlessSparkWithPID sparkLiftR;
    public static BrushlessSparkWithPID sparkClawTilt;
    public static BrushlessSparkWithPID sparkIntake;
    //public static AHRS ahrs;

    
    public static AnalogPotentiometer frontLiftPot;
    public static AnalogPotentiometer rearLiftPot;
    public static DigitalInput intakeSwitch;

    

    public static void init(){
        //Create new instance of each component.
        
        sparkLiftF = new BrushlessSparkWithPID(9, 0.0006, 0.0000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 4000, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc,Constants.PID.Wheels.allowedErr);
        sparkLiftR = new BrushlessSparkWithPID(10, 0.0008, 0.00001, 0.001, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 4000, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        frontLiftPot = new AnalogPotentiometer(4, 3.48, -0.2);//(4+0, 3.48,-0.2);
        rearLiftPot = new AnalogPotentiometer(4+1, 3.58,-0.3);
        intakeSwitch = new DigitalInput(4);


        //Timmy Code Probably Bad!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Timmy Code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Timmy Code!!!!!!!!!!!!!!!!

        //sparkClawTilt = new BrushlessSparkWithPID(4, 0.0005, 0.000002, 0.0015, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkIntake = new BrushlessSparkWithPID(3, 0.000006, 0.000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkClawTilt = new BrushlessSparkWithPID(4, 0.0005, 0.000001, 0.0015, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, 0.03);

        //End Timmy Code!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Back to Safety End Timmy Code!!!!!!!!!!!!

    }

}
