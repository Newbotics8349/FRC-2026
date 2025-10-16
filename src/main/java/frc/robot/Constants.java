package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    // TODO: Change to correct values for all
    public static final class DriveConstants {
        public static final Translation2d kFrontLeftLocation = new Translation2d(1, 1);
        public static final Translation2d kFrontRightLocation = new Translation2d(1, -1);
        public static final Translation2d kBackRightLocation = new Translation2d(-1, -1);
        public static final Translation2d kBackLeftLocation = new Translation2d(-1, 1);

		public static final double kTeleOpMaxAcceleration = 1;
		public static final double kTeleOpMaxAngularAcceleration = 1;
        public static final double kTeleOpMaxMetersPerSecond = 5;
        public static final double kTeleOpMaxAngularMetersPerSecond = 3;
    }

    public static final class ModuleConstants {
        public static final class FrontLeft {
            public static final int kDriveMotorId = 0;
            public static final int kTurnMotorId = 0;
            public static final int kTurnEncoderId = 0;  
        }
        public static final class FrontRight {
            public static final int kDriveMotorId = 0;
            public static final int kTurnMotorId = 0;
            public static final int kTurnEncoderId = 0;              
        }
        public static final class BackRight {
            public static final int kDriveMotorId = 0;
            public static final int kTurnMotorId = 0;
            public static final int kTurnEncoderId = 0;  
        }
        public static final class BackLeft {
            public static final int kDriveMotorId = 0;
            public static final int kTurnMotorId = 0;
            public static final int kTurnEncoderId = 0;            
        }
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class ControllerConstants {
        public static final int kDriverController = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonId = 1;

        public static final double kDeadband = 0.05;
    }

}
