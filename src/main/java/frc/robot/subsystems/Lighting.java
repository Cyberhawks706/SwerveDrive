package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
    public static DigitalOutput out0, out1, out2, out3;
    
    public static void robotInit() {
        out0 = new DigitalOutput(0);
        out1 = new DigitalOutput(1);
        out2 = new DigitalOutput(2);
        out3 = new DigitalOutput(3);
        out0.set(false);
        out1.set(false);
        out2.set(false);
        out3.set(false);
    }
    
    public static void setLEDS(String color) {
        switch (color) {
            case "BLACK":
                setLEDPins(new int[]{0,0,0,0});
                break;
            case "PINK":
                setLEDPins(new int[]{1,1,1,1});
                break;
            case "YELLOW":
                setLEDPins(new int[]{1,0,0,0});
                break;
            case "PURPLE":
                setLEDPins(new int[]{0,1,0,0});
                break;
            case "RED":
                setLEDPins(new int[]{0,0,1,0});
                break;
            case "BLUE":
                setLEDPins(new int[]{0,0,0,1});
                break;
            default:
                break;

        }
        setLEDPins(new int[]{0,0,0,0});
    }
    public static void setLEDPins(int[] pins) {
        boolean pinStates[] = new boolean[4];
        for (int i = 0; i < 4; i++) {
            pinStates[i] = pins[i] == 1 ? true : false;
        }
        out0.set(pinStates[0]);
        out1.set(pinStates[1]);
        out2.set(pinStates[2]);
        out3.set(pinStates[3]);
    }
}
