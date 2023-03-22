package frc.robot;

import java.util.Timer;

//import com.kauailabs.navx.frc.AHRS;

import frc.robot.config.Config;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.DashboardDaemon;
//import frc.robot.subsystems.PIDDaemon;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.Clock;
import frc.robot.commands.LimelightTrack;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;



public class Components {

    static public Chassis chassis;
    static public Limelight limelight;
    public static NetworkTableInstance networkTableInstance;
    public static NetworkTable visionTable;
    public static Config configtable;
    public static DashboardDaemon daemon;
    //public static PIDDaemon PIDDaemon;
    public static PIDController turnController;
    public static LimelightTrack limelightTrack;
    

    public static BrushlessSparkWithPID sparkWheelFR;
    public static BrushlessSparkWithPID sparkWheelFL;
    public static BrushlessSparkWithPID sparkWheelBR;
    public static BrushlessSparkWithPID sparkWheelBL;
    public static BrushlessSparkWithPID sparkWheelTurnFR;
    public static BrushlessSparkWithPID sparkWheelTurnFL;
    public static BrushlessSparkWithPID sparkWheelTurnBR;
    public static BrushlessSparkWithPID sparkWheelTurnBL;
    public static BrushlessSparkWithPID sparkLiftF;
    public static BrushlessSparkWithPID sparkLiftR;
    public static BrushlessSparkWithPID sparkClawTilt;
    public static BrushlessSparkWithPID sparkIntake;
    //public static AHRS ahrs;
   
    public static Timer timer;
    public static Clock ClimbClock;
    
    public static AnalogPotentiometer frontLiftPot;
    public static AnalogPotentiometer rearLiftPot;
    public static DigitalInput intakeSwitch;

    

    public static void init(){
        //Create new instance of each component.
        
        
        timer = new Timer();

        
        
        daemon = new DashboardDaemon();
        //PIDDaemon = new PIDDaemon();
        configtable = new Config();
        networkTableInstance = NetworkTableInstance.getDefault();

        //xboxDrive = new XboxController(2);
        chassis = new Chassis();
        limelight = new Limelight();
        
        //   sparkLiftF = new BrushlessSparkWithPID(9, 0.0006, 0.000001, 0.0001, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        //sparkLiftR = new BrushlessSparkWithPID(10, 0.0006, 0.000001, 0.0001, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        //frontLiftPot = new AnalogPotentiometer(4+0, 3.48,-0.2);
        //rearLiftPot = new AnalogPotentiometer(4+1, 3.58,-0.3);


        //sparkLiftF = new BrushlessSparkWithPID(9, 0.0006, 0.000001, 0.0003, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        //sparkLiftR = new BrushlessSparkWithPID(10, 0.0006, 0.000001, 0.0003, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        //frontLiftPot = new AnalogPotentiometer(4, 3.48, -0.2);//(4+0, 3.48,-0.2);
        sparkLiftF = new BrushlessSparkWithPID(9, 0.0006, 0.0000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 4000, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc,Constants.PID.Wheels.allowedErr);

        //0.001, 0.0000001, 0.0002
        sparkLiftR = new BrushlessSparkWithPID(10, 0.0006, 0.000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, 4000, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        frontLiftPot = new AnalogPotentiometer(4, 3.48, -0.2);//(4+0, 3.48,-0.2);
        rearLiftPot = new AnalogPotentiometer(4+1, 3.58,-0.3);
        intakeSwitch = new DigitalInput(4);

        System.out.println(frontLiftPot.get());
        

        //Timmy Code Probably Bad!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Timmy Code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Timmy Code!!!!!!!!!!!!!!!!

        //sparkClawTilt = new BrushlessSparkWithPID(4, 0.0005, 0.000002, 0.0015, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkIntake = new BrushlessSparkWithPID(3, 0.000006, 0.000001, 0.0005, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkClawTilt = new BrushlessSparkWithPID(4, 0.0005, 0.000001, 0.0015, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, 0.03);

        //End Timmy Code!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Back to Safety End Timmy Code!!!!!!!!!!!!

        //ahrs = new AHRS();
        //ahrs.reset();
        //System.out.println("hello");

    
        ClimbClock = new Clock(Constants.TimingArrays.climbStateLengths);


        
      
        
    }

}
