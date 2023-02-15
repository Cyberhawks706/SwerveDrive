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


    public static BrushlessSparkWithPID frontLiftMotor;
    public static BrushlessSparkWithPID backLiftMotor;


    //public static AHRS ahrs;
   
    public static Timer timer;
    public static Clock ClimbClock;
    


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
        limelightTrack = new LimelightTrack(limelight);

        frontLiftMotor = new BrushlessSparkWithPID(10, 2.1e-4, 0, 0, 0.000156, 0, 1, -1, 2000, 0, 1500, 0);
        backLiftMotor = new BrushlessSparkWithPID(9, 2.1e-4, 0, 0, 0.000156, 0, 1, -1, 2000, 0, 1500, 0);

        
        

        

        //ahrs = new AHRS();
        //ahrs.reset();

    
        ClimbClock = new Clock(Constants.TimingArrays.climbStateLengths);
        
      
        
    }

}
