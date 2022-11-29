package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class IO {
    //static final public Joystick rightJoy = new Joystick(Constants.IO.RIGHT_JOY);
    //static final public Joystick leftJoy = new Joystick(Constants.IO.LEFT_JOY);
    static final public XboxController xboxManipulator = new XboxController(Constants.IO.XBOXMANIP);
    static final public XboxController xboxDrive = new XboxController(Constants.IO.XBOXDRIVE);
    public void init() {
    }
    
//   public JoystickButton a = new JoystickButton(xbox, Constants.IO.A), b = new JoystickButton(xbox, Constants.IO.Y);
    //static public XboxController xboxliftcontroller = new XboxController(Constants.IO.XBOXDRIVE);;
    /*
    public void init() {
        xboxliftcontroller = new XboxController(Constants.IO.XBOX);
    }*/
}