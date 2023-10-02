package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class ArmConstants{

        // TODO: ADJUST CONSTANTS
        public static final int rotateMotorId = 20;
        public static final int extensionMotorId = 21;
        public static final int rotateCanCoderId = 22;

        public static final double rotatekP = 0;
        public static final double rotatekI = 0;
        public static final double rotatekD = 0;

        public static final double extensionkP = 0;
        public static final double extensionkI = 0;
        public static final double extensionkD = 0;

    }

    public static final class DriveConstants{

        public static final double kWheelBase = 23.5;
        public static final double kTrackWidth = 16;

        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kMaxAccelerationRateUnitsPerSecond = 5.0;
        public static final double kMaxTurningRateUnitsPerSecond = 5.0;

        public static final double kDriveMaxSpeedMetersPerSecond = 1;
        public static final double kDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    }

    public static final class ModuleConstants{

        public static final double kModuleP = 0.1;
        public static final double kModuleI = 0;
        public static final double kModuleD = 0;

        public static final double kAllowableError = 0;
        public static final double kDriveCurrentLimit = 0;
        public static final double kTurnCurrentLimit = 0;

        public static final class FrontLeftModule{
            public static final int driveMotorId = 1;
            public static final int turnMotorId = 2;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 3;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final String name = "Front Left";
        }

        public static final class FrontRightModule{
            public static final int driveMotorId = 4;
            public static final int turnMotorId = 5;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 6;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final String name = "Front Right";
        }

        public static final class BackLeftModule{
            public static final int driveMotorId = 7;
            public static final int turnMotorId = 8;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 9;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final String name = "Back Left";
        }

        public static final class BackRightModule{
            public static final int driveMotorId = 10;
            public static final int turnMotorId = 11;
            public static final boolean driveMotorReversed = false;
            public static final boolean turnMotorReversed = false;
            public static final int turnCanCoderId = 12;
            public static final double absoluteEncoderOffsetRad = 0;
            public static final String name = "Back Right";
        }

    }

    public static final class IOConstants{

        public static final double kDeadband = 0.05;
    }
}