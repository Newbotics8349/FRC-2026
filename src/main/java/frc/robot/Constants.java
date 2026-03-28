package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final class DriveConstants {
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.265, 0.265);
        public static final Translation2d kFrontRightLocation = new Translation2d(0.265, -0.265);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.265, -0.265);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.265, 0.265);

		public static final double kTeleOpMaxAcceleration = 5;
		public static final double kTeleOpMaxAngularAcceleration = 3;
        public static final double kTeleOpMaxMetersPerSecond = 0.75;
        public static final double kTeleOpMaxAngularMetersPerSecond = 0.5;
        public static final double kTeleOpMinMetersPerSecond = 0.05;
        public static final double kTeleOpMinAngularMetersPerSecond = 0.05;

        public static final int kGyroId = 0;
    }

    public static final class ModuleConstants {
        public static final class FrontLeft {
            public static final int kDriveMotorId = 21;
            public static final int kTurnMotorId = 11;
            public static final int kTurnEncoderId = 1;
            public static final boolean driveInverted = false;
        }
        public static final class FrontRight {
            public static final int kDriveMotorId = 22;
            public static final int kTurnMotorId = 12;
            public static final int kTurnEncoderId = 2;     
            public static final boolean driveInverted = true;    
        }
        public static final class BackRight {
            public static final int kDriveMotorId = 23;
            public static final int kTurnMotorId = 13;
            public static final int kTurnEncoderId = 3;
            public static final boolean driveInverted = true;
        }
        public static final class BackLeft {
            public static final int kDriveMotorId = 24;
            public static final int kTurnMotorId = 14;
            public static final int kTurnEncoderId = 4;   
            public static final boolean driveInverted = false;
        }
        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0.002;

        public static final double wheelDiameterInches = 4;
        public static final double driveGearRatio = 6.75;
    }

    public static final class ControllerConstants {
        public static final int kDriverController = 0;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverSlowAxis = 3;
        public static final int kDriverRotAxis = 4;

        public static final int kDriverFieldOrientedButtonId = 5;
        public static final int kDriverResetGyroId = 2;

        public static final double kDeadband = 0.05;
    }

    public static final class ShooterConstants {
        public static final int vout = 400;
        public static final double g = 386.09;
        
        public static final int turretEncoderPort = 100;
        public static final int hoodEncoderPort = 101;

        public static final double turretGearRatio = 5 * 154 / 16;

        public static final int turretMotorId = 41;
        public static final int hoodMotorId = 46;

        public static final double[] turretPID = {
            0.6, 0, 0
        };
        public static final double[] hoodPID = {
            0.4, 0.1, 0.03
        };

        public static final int[] shooterMotorIds = {31, 32};
        public static final double shooterMotorSpeed = 0.6;

        public static final Transform3d turretOffset = new Transform3d(-12.5, -8.7, 0, new Rotation3d(new Rotation2d(Math.PI / 2)));
    }

    public static final class IntakeConstants {
        public static int omniMotorId = 44;
        public static int intakeMotorId = 42;
        public static int feederMotorId = 40;
        public static int extensionMotorId = 43;
        public static int conveyorPWMPort = 9;

        public static double omniMotorSpeed = 0.15;
        public static double intakeMotorSpeed = -0.65;
        public static double feederMotorSpeed = -0.7;
        public static double conveyorMotorSpeed = 0.75;

        public static double[] extensionPID = {
            0.035, 0, 0
        };
        public static int extensionEncoderPort = 105;
        public static double extensionOut = 5;
        public static double extensionIn = 0;
    }
    
    public static final class PWMConstants {
        public static final int kLedPort = 0;
    }
    
    public static final int numLeds = 60;

    public static enum AprilTag {
        Hub1(8, new Transform3d(23.5, 0, 0, new Rotation3d(0, 0, Math.PI)));

        public final int id;
        public final Transform3d offset;

        private AprilTag(int id, Transform3d offset) {
            this.id = id;
            this.offset = offset;
        }
    }
}
