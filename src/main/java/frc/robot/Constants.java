package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public static final class DriveConstants {
        public static final Translation2d kFrontLeftLocation = new Translation2d(1, 1);
        public static final Translation2d kFrontRightLocation = new Translation2d(1, -1);
        public static final Translation2d kBackRightLocation = new Translation2d(-1, -1);
        public static final Translation2d kBackLeftLocation = new Translation2d(-1, 1);

		public static final double kTeleOpMaxAcceleration = 5;
		public static final double kTeleOpMaxAngularAcceleration = 3;
        public static final double kTeleOpMaxMetersPerSecond = 0.5;
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
        
    }

    public static final class PWMConstants {
        public static final int kLedPort = 0;
    }

    public static final int numLeds = 60;

    public static enum AprilTag {
        Test(21, new Transform3d(1, 0, 0, new Rotation3d(0, 0, Math.PI))),
        Test2(20, new Transform3d(2, 0.5, 0, new Rotation3d(0, 0, Math.PI)));

        public final int id;
        public final Transform3d offset;

        private AprilTag(int id, Transform3d offset) {
            this.id = id;
            this.offset = offset;
        }
    }
}
