package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final int PID_TIMEOUT = 30;

	public static final class Auton {
		public static final double strafeModifier = 1;
		public static final double rotationalModifier = 1;
		public static final double tolerance = 2;
		public static final Transform3d[] fieldRelativeTagPositions = {
				new Transform3d(), // offset so we can just look up transform by tag id
				new Transform3d(new Translation3d(7.243064, -2.936494, 0), new Rotation3d(0, 0, 180)),
				new Transform3d(new Translation3d(7.243064, -1.260094, 0), new Rotation3d(0, 0, 180)),
				new Transform3d(new Translation3d(7.243064, 0.416306, 0), new Rotation3d(0, 0, 180)),
				new Transform3d(new Translation3d(7.90829, 2.741676, 0), new Rotation3d(0, 0, 180)),
				new Transform3d(new Translation3d(-7.908544, 2.741676, 0), new Rotation3d(0, 0, 0)),
				new Transform3d(new Translation3d(-7.243064, 0.416306, 0), new Rotation3d(0, 0, 0)),
				new Transform3d(new Translation3d(-7.243064, -1.260094, 0), new Rotation3d(0, 0, 0)),
				new Transform3d(new Translation3d(-7.243064, -2.936494, 0), new Rotation3d(0, 0, 0))
		};

		// public static final int balanceReverseDelay = 385; //210
		public static final int balanceReverseDelay = 180;// 220 //385 works with back divisor of 40
	}


	public class Drive {
		public static final double PI = 3.2;
		public static final double ROT_SIZE = 12.8; // length of 1 full rotation
		public static final double PWR_MODIFIER = 0.25;
	}
	//all the drive stuff
	public static final class Swerve {
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Absolute Encoder Values

		public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
		public static final double kFrontLeftDriveAbsoluteEncoderOffsetrad = 2.07;// -3.75
		public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

		public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
		public static final double kBackLeftDriveAbsoluteEncoderOffsetrad = 0.03;// 0.328
		public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

		public static final int kBackRightDriveAbsoluteEncoderPort = 2;
		public static final double kBackRightDriveAbsoluteEncoderOffsetrad = -2.7;// -0.6
		public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

		public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
		public static final double kFrontRightDriveAbsoluteEncoderOffsetrad = 4.07;// -0.122//0.32
		public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Module Constants
		
		public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
		public static final double kDriveMotorGearRatio = 1 / 8.14;
		public static final double kTurningMotorGearRatio = 1 / 12.8;
		public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
		public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
		public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
		public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
		public static final double kPTurning = 0.5;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Drive Constants

		public static final double kTrackWidth = Units.inchesToMeters(22.5);
		// Distance between right and left wheels
		public static final double kWheelBase = Units.inchesToMeters(22.5);
		// Distance between front and back wheels
		
		public static final int kFrontLeftDriveMotorPort = 6;
		public static final int kBackLeftDriveMotorPort = 1;
		public static final int kFrontRightDriveMotorPort = 11;
		public static final int kBackRightDriveMotorPort = 8;

		public static final int kFrontLeftTurningMotorPort = 5;
		public static final int kBackLeftTurningMotorPort = 2;
		public static final int kFrontRightTurningMotorPort = 12;
		public static final int kBackRightTurningMotorPort = 7;

		public static final boolean kFrontLeftTurningEncoderReversed = false;
		public static final boolean kBackLeftTurningEncoderReversed = false;
		public static final boolean kFrontRightTurningEncoderReversed = false;
		public static final boolean kBackRightTurningEncoderReversed = false;

		public static final boolean kFrontLeftDriveEncoderReversed = false;
		public static final boolean kBackLeftDriveEncoderReversed = true;
		public static final boolean kFrontRightDriveEncoderReversed = false;
		public static final boolean kBackRightDriveEncoderReversed = false;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
				kPhysicalMaxAngularSpeedRadiansPerSecond;
		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Auto Constants

		public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
		public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
		public static final double kPXController = 1.5;
		public static final double kPYController = 1.5;
		public static final double kPThetaController = 3;

		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
				new TrapezoidProfile.Constraints(
						kMaxAngularSpeedRadiansPerSecond,
						kMaxAngularAccelerationRadiansPerSecondSquared);

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// OI Constants

		public static final int kDriverControllerPort = 2;

		public static final double kDeadband = 0.05;

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

		// Xbox deadband
		public static final double xboxDeadband = 0.08;
	}

	public class PID {
		// PID coefficients
		public class Wheels {
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
			public static final double maxAcc = 1000;

			public static final double allowedErr = 0;
		}

		public class TurnMotor {
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
			public static final double maxAcc = 5000;

			public static final double allowedErr = 0;
		}

	}

}
