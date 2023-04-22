package frc.robot.config;

public final class Config {

   public static ConfigurableBool testBool = new ConfigurableBool("TESTBOOL", true);
   public static ConfigurableNumber testNumber = new ConfigurableNumber("TESTNUMBER", 0);
   public static ConfigurableString testString = new ConfigurableString("TESTSTRING", "TRUE");

   //static double angle = Components.ahrs.getAngle();
   //public static ConfigurableNumber angleNumber  = new ConfigurableNumber("Angle", angle);
   }