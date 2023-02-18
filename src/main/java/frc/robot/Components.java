package frc.robot;

import java.util.Timer;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.config.Config;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.DashboardDaemon;
import frc.robot.subsystems.PIDDaemon;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Clock;
import frc.robot.commands.Drive;
import frc.robot.commands.LimelightTrack;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Joystick;


public class Components {

    static public Chassis chassis;
    public static NetworkTableInstance networkTableInstance;
    public static NetworkTable visionTable;
    public static Config configtable;
    public static DashboardDaemon daemon;
    //public static PIDDaemon PIDDaemon;
    public static PIDController turnController;
    public static Drive drive;
    public static LimelightTrack limelighttrack;

    public static BrushlessSparkWithPID sparkWheelFR;
    public static BrushlessSparkWithPID sparkWheelFL;
    public static BrushlessSparkWithPID sparkWheelBR;
    public static BrushlessSparkWithPID sparkWheelBL;
    public static BrushlessSparkWithPID sparkWheelTurnFR;
    public static BrushlessSparkWithPID sparkWheelTurnFL;
    public static BrushlessSparkWithPID sparkWheelTurnBR;
    public static BrushlessSparkWithPID sparkWheelTurnBL;
    public static AHRS ahrs;
   
    public static Timer timer;
    public static Clock ClimbClock;
    public static SwerveSubsystem swerveSubsystem;
    public static Joystick driverJoystick;
    private static PIDDaemon PIDDaemon;


    public static void init(){
        //Create new instance of each component.
        
        
        timer = new Timer();

        chassis = new Chassis();
        swerveSubsystem = new SwerveSubsystem();
        driverJoystick = new Joystick(2);

        
        daemon = new DashboardDaemon();
        PIDDaemon = new PIDDaemon();
        configtable = new Config();
        networkTableInstance = NetworkTableInstance.getDefault();
        drive = new Drive();
        limelighttrack = new LimelightTrack();

        ahrs = new AHRS();
        ahrs.reset();

        /* 
        sparkWheelFR = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelFR, Constants.PID.Wheels.kP, Constants.PID.Wheels.kI, Constants.PID.Wheels.kD, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkWheelFL = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelFL, Constants.PID.Wheels.kP, Constants.PID.Wheels.kI, Constants.PID.Wheels.kD, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkWheelBR = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelBR, Constants.PID.Wheels.kP, Constants.PID.Wheels.kI, Constants.PID.Wheels.kD, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkWheelBL = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelBL, Constants.PID.Wheels.kP, Constants.PID.Wheels.kI, Constants.PID.Wheels.kD, Constants.PID.Wheels.kFF, Constants.PID.Wheels.kIz, Constants.PID.Wheels.kMinOutput, Constants.PID.Wheels.kMaxOutput, Constants.PID.Wheels.maxVel, Constants.PID.Wheels.minVel, Constants.PID.Wheels.maxAcc, Constants.PID.Wheels.allowedErr);
        sparkWheelTurnFR = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelTurnFR, Constants.PID.TurnMotor.kP, Constants.PID.TurnMotor.kI, Constants.PID.TurnMotor.kD, Constants.PID.TurnMotor.kFF, Constants.PID.TurnMotor.kIz, Constants.PID.TurnMotor.kMinOutput, Constants.PID.TurnMotor.kMaxOutput, Constants.PID.TurnMotor.maxVel, Constants.PID.TurnMotor.minVel, Constants.PID.TurnMotor.maxAcc, Constants.PID.TurnMotor.allowedErr);
        sparkWheelTurnFL = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelTurnFL, Constants.PID.TurnMotor.kP, Constants.PID.TurnMotor.kI, Constants.PID.TurnMotor.kD, Constants.PID.TurnMotor.kFF, Constants.PID.TurnMotor.kIz, Constants.PID.TurnMotor.kMinOutput, Constants.PID.TurnMotor.kMaxOutput, Constants.PID.TurnMotor.maxVel, Constants.PID.TurnMotor.minVel, Constants.PID.TurnMotor.maxAcc, Constants.PID.TurnMotor.allowedErr);
        sparkWheelTurnBR = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelTurnBR, Constants.PID.TurnMotor.kP, Constants.PID.TurnMotor.kI, Constants.PID.TurnMotor.kD, Constants.PID.TurnMotor.kFF, Constants.PID.TurnMotor.kIz, Constants.PID.TurnMotor.kMinOutput, Constants.PID.TurnMotor.kMaxOutput, Constants.PID.TurnMotor.maxVel, Constants.PID.TurnMotor.minVel, Constants.PID.TurnMotor.maxAcc, Constants.PID.TurnMotor.allowedErr);
        sparkWheelTurnBL = new BrushlessSparkWithPID(Constants.SparkIDs.sparkWheelTurnBL, Constants.PID.TurnMotor.kP, Constants.PID.TurnMotor.kI, Constants.PID.TurnMotor.kD, Constants.PID.TurnMotor.kFF, Constants.PID.TurnMotor.kIz, Constants.PID.TurnMotor.kMinOutput, Constants.PID.TurnMotor.kMaxOutput, Constants.PID.TurnMotor.maxVel, Constants.PID.TurnMotor.minVel, Constants.PID.TurnMotor.maxAcc, Constants.PID.TurnMotor.allowedErr);
        */

        

        ClimbClock = new Clock(Constants.TimingArrays.climbStateLengths);
        
      
        
    }

}
