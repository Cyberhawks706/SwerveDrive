package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
     ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     //Absolute Encoder Values

     public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
     public static final double kFrontLeftDriveAbsoluteEncoderOffsetrad = 0;
     public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

     public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
     public static final double kBackLeftDriveAbsoluteEncoderOffsetrad = 1.74;
     public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

     public static final int kBackRightDriveAbsoluteEncoderPort = 2;
     public static final double kBackRightDriveAbsoluteEncoderOffsetrad = -2.64;
     public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

     public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
     public static final double kFrontRightDriveAbsoluteEncoderOffsetrad = 0;
     public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;





    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Module Constants
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/10000000;
    public static final double kTurningMotorGearRatio = 1/12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Drive Constants

    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

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
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Auto Constants

    public static final double kMaxSpeedMetersPerSecond = SwerveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                SwerveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
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
    //OI Constants

        public static final int kDriverControllerPort = 2;

        public static final double kDeadband = 0.05;


}
