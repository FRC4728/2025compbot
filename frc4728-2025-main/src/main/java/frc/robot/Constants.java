package frc.robot;
public final class Constants {

    public static final class limelightConstants {
        public static String leftLimelight = "limelight-left";
        public static String rightLimelight = "limelight-right";
        public static double defaultValue = 0.0;
    }

    public static final class minMaxOutputConstants {
        /* Define max and min outputs */
        public static final int maxOutputRPM = 5700;
        public static final int kMaxOutput = 1;
        public static final int kMinOutput = -1;
    }

    public static final class ElevatorConstants {
        /*Define elevator PID controller variables and motor IDs */
        public static double kP = 4.8;
        public static double kI = 0.0;
        public static double kD = 0.1;
        public static double kS = 0.25;
        public static double kV = 0.12;
        public static double kA = 0.01;

        public static double kAcceleration = 220; //Was 120
        public static double kCruiseVelo = 120; //Was 110
        public static double kJerk = 1600.0;
        public static double kMotionMagicV = 0.12;
        public static double kMotionMagicA = 0.1;

        public static double currentLimit = 50.0;
        public static boolean enableCurrentLimits = true;

        public static int topMotorID = 5;
        public static int bottomMotorID = 9;
    }

    public static final class IntakeConstants {
        public static double kP = 0.1;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kS = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;

        public static double runIntakeInVelo = -30;
        public static double runIntakeOutVelo = 27;
        public static double runIntakeScoreVelo = 45;

        public static int motor1ID = 8;
        public static int photoCellDIO = 7;
    }
    public static final class WristConstants {
        /*Define wrist controller variables and motor IDs */
        public static double kP = 4.8;
        public static double kI = 0.0;
        public static double kD = 0.1;
        public static double kS = 0.25;
        public static double kV = 0.12;
        public static double kA = 0.01;

        public static double kAcceleration = 100.0;
        public static double kCruiseVelo = 70.0;

        public static double currentLimit = 45.0;
        public static boolean enableCurrentLimits = true;

        public static int motor1ID = 32;

        public static int cancoderID = 40;
    }

    public static final class ClimberConstants {
        /*Define climber controller variables and motor IDs(s) */
        public static double kP = 0.0001;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kFF = 0.5;

        public static int kMaxMotionVelocity = 500;
        public static int kMaxMotionAcceleration = 3000;

        public static int climberMotorID = 30;

        public static double climberFwdVelo = 90;
        public static double climberRevVelo = -90;

    }
}
