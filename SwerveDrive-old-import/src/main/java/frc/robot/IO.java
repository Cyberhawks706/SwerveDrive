package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

//import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class IO {
    
    //static final public XboxController xboxManipulator = new XboxController(Constants.IO.XBOXMANIP);
    static public XboxController xboxDrive;
    public void init() {

        
        xboxDrive = new XboxController(2);
        System.out.println("Reaching" + xboxDrive);
    }
    
//   public JoystickButton a = new JoystickButton(xbox, Constants.IO.A), b = new JoystickButton(xbox, Constants.IO.Y);
    //static public XboxController xboxliftcontroller = new XboxController(Constants.IO.XBOXDRIVE);;

}