package frc.robot.config;



import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Components;
import frc.robot.commands.SwerveJoystickCmd;

public final class Config {

   //public static ConfigurableBool testBool = new ConfigurableBool("TESTBOOL", true);
   //public static ConfigurableNumber testNumber = new ConfigurableNumber("TESTNUMBER", 0);
   //public static ConfigurableString testString = new ConfigurableString("TESTSTRING", "TRUE");
   public static ConfigurableNumber inputXvalue = new ConfigurableNumber("inputXvalue", 0);
   public static ConfigurableNumber outputSpeed = new ConfigurableNumber("outputSpeed", 0);

   //static double angle = Components.ahrs.getAngle();
   //public static ConfigurableNumber angleNumber  = new ConfigurableNumber("Angle", angle);
   }