package frc.robot.config;



import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Components;

public final class Config {

   public static ConfigurableBool testBool = new ConfigurableBool("TESTBOOL", true);
   public static ConfigurableNumber testNumber = new ConfigurableNumber("TESTNUMBER", 0);
   public static ConfigurableString testString = new ConfigurableString("TESTSTRING", "TRUE");

   //static double angle = Components.ahrs.getAngle();
   //public static ConfigurableNumber angleNumber  = new ConfigurableNumber("Angle", angle);
   }