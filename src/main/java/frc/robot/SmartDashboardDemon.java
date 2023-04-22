package frc.robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SmartDashboardDemon {
    
    NetworkTableEntry entry1;
    NetworkTableEntry entry2;

    NetworkTableInstance testNTInstance = NetworkTableInstance.getDefault();;
    NetworkTable testTable = testNTInstance.getTable("datatable");

    
    double x, y;
    public void updateValues(){
        entry1.setDouble(x);
        entry2.setDouble(y);

        x += 0.05;
        y += 1.0;

        
    }

    

}
