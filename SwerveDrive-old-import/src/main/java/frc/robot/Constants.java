package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {

	public static class TimingArrays {
		public static final double[] climbStateLengths = new double[] {1,1,1,1,1,1,1,1,1,1,1,1};

	}

	

	public static final int PID_TIMEOUT = 30;

	public static class Auton {
		public static final double strafeModifier = 1;
		public static final double rotationalModifier = 1;
		public static final double tolerance = 2;
		public static final Transform3d[] fieldRelativeTagPositions = {
			new Transform3d(),//offset so we can just look up transform by tag id
			new Transform3d(new Translation3d(7.243064, -2.936494,0), new Rotation3d(0,0,180)),
			new Transform3d(new Translation3d(7.243064, -1.260094,0), new Rotation3d(0,0,180)),
			new Transform3d(new Translation3d(7.243064, 0.416306,0), new Rotation3d(0,0,180)),
			new Transform3d(new Translation3d(7.90829, 2.741676,0), new Rotation3d(0,0,180)),
			new Transform3d(new Translation3d(-7.908544, 2.741676,0), new Rotation3d(0,0,0)),
			new Transform3d(new Translation3d(-7.243064, 0.416306,0), new Rotation3d(0,0,0)),
			new Transform3d(new Translation3d(-7.243064,-1.260094,0), new Rotation3d(0,0,0)),
			new Transform3d(new Translation3d(-7.243064,-2.936494,0), new Rotation3d(0,0,0))
		};
		public static final int balanceReverseDelay = 230; //210
	}

	public class Chassis {
		//ALL CAN IDs
	
		// Driving
		public static final boolean USE_XBOX = false;
		public static final boolean LOGGING_ENABLED = false;
		public static final boolean JOYDEADZONE = true; //enables both rotation and xy deadzone
		//Driving Constants

	}

	public class Drive {
		public static final double PI = 3.2;
		public static final double ROT_SIZE = 12.8; //length of 1 full rotation
		public static final double PWR_MODIFIER = 0.25;
	}

	public class IO {
		// Joystick Ports
		public static final int LEFT_JOY = 1;
		public static final int RIGHT_JOY = 0;
		public static final int XBOXDRIVE = 2;
		public static final int XBOXMANIP = 3;

		// Joystick Buttons
		public static final int TRIGGER = 1;
		public static final int CLIMB = 2;

		// Xbox Buttons
		public static final int A = 1;
		public static final int B = 2;
		public static final int X = 3;
		public static final int Y = 4;
		public static final int LB = 5;
		public static final int RB = 6;
		public static final int BACK = 7;
		public static final int START = 8;
		public static final int MANIP = 9;
		public static final int VISION_TRIGGER = 2;

		//Xbox deadband
		public static final double xboxDeadband = 0.08;
	}

	
	public class PID{
		// PID coefficients
		public class Wheels{
			public static final double kP = 2.1e-4; 
			public static final double kI = 0;
			public static final double kD = 0; 
			public static final double kIz = 0; 
			public static final double kFF = 0.000156; 
			public static final double kMaxOutput = 1; 
			public static final double kMinOutput = -1;
			public static final double maxRPM = 5700;
		
			// Smart Motion Coefficients
			public static final double maxVel = 2000; // rpm
			public static final double minVel = 0; // rpm
			public static final double 	maxAcc = 1500;

			public static final double allowedErr = 0;
		}

		public class TurnMotor{
			public static final double kP = 1.5e-4; 
			public static final double kI = 3.5e-10;
			public static final double kD = 5.5e-5; 
			public static final double kIz = 0; 
			public static final double kFF = 0.000156; 
			public static final double kMaxOutput = 1; 
			public static final double kMinOutput = -1;
			public static final double maxRPM = 5700;
		
			// Smart Motion Coefficients
			public static final double maxVel = 9000; // rpm
			public static final double minVel = 0; // rpm
			public static final double 	maxAcc = 5000;

			public static final double allowedErr = 0;
		}

	}

	
}
