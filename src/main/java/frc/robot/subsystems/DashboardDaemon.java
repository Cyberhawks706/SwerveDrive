package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;


// All cower in fear before the dashboard demon

public final class DashboardDaemon extends Subsystem {

        { // You can set callbacks here
                /*
                 * EXAMPLE
                 * Config.driveInBrakeMode.onChangeCallback = () -> {
                 * IdleMode mode = IdleMode.kCoast;
                 * if (Config.driveInBrakeMode.value == true) {
                 * mode = IdleMode.kBrake;
                 * }
                 * 
                 * };
                 */
        }

        public void initDefaultCommand() {
                // No default command, is a daemon

        }

        public void periodic() {
                // !! Use DriverStation.reportWarning("MESSAGE", false); when printing warning

                // IN ORDER TO PRINT VALUES IN DRIVERSTATION DASHBOARD FOLLOW THESE STEPS:
                // 1) Initialize value in Config. Determine if boolean or number
                // 2) Config.------.pull/push
        

        }

}

/*
 * 
 * ~~ THE DASHBOARD DEMON ~~
 * 
 * ~ . ...... . ~
 * ~ . . ~
 * ~. __ __ .~
 * . @@@ @@@ .
 * . / .
 * . / .
 * . / .
 * . * ---- * .
 * . * * .
 * . ******* .
 * . .
 * ...
 * 
 */