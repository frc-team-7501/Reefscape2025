package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final class CANMapping {

        // DriveTrain 2023 Bot
        public static final int TALONFX_DRIVE_BL = 52;
        public static final int TALONFX_DRIVE_BR = 53;
        public static final int TALONFX_DRIVE_FL = 54;
        public static final int TALONFX_DRIVE_FR = 51;

        public static final int SPARKMAX_TURN_BL = 32;
        public static final int SPARKMAX_TURN_BR = 33;
        public static final int SPARKMAX_TURN_FL = 34;
        public static final int SPARKMAX_TURN_FR = 31;

        public static final int TURN_CANCODER_BL = 42;
        public static final int TURN_CANCODER_BR = 43;
        public static final int TURN_CANCODER_FL = 44;
        public static final int TURN_CANCODER_FR = 41;

        // Other Motors

        // MISC CAN Bus
        public static final int PIGEON_IMU = 20;
    }

    public static final class ControllerMapping {
        public static final int XBOX = 0;
        public static final int XBOX2 = 1;
    }

    public static final class DIOMapping {}

    public static final class MiscMapping {
        public static final boolean BRAKE_ON = true;
        public static final boolean BRAKE_OFF = false;
        public static final boolean FIELD_RELATIVE = true;
        public static final double MAXSPEED = 1;
        public static final double MAXANGULARSPEED = 1.4;
        public static final double NORMAL_MULTIPLIER = 1;
        public static final double TURBO_MULTIPLIER = 1;
        public static final double PHOTON_PITCH_GOAL = 16;
        public static final double xConversionInches = 39.2 / 77; // 39.2 units / 77 inches
        public static final double yConversionInches = -39.47 / 78; // negate so left is negative, right is positive
    }

    public static final class TalonMapping {
        public static final double PID_P = 0.11;// An error of 1 rotation per second results in 2V output

        public static final double PID_I = 0.5; // An error of 1 rotation per second increases output by 0.5V every
                                                // second
        public static final double PID_D = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts
                                                   // output
        public static final double PID_V = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33
                                                 // = 0.12 volts / Rotation per second
        public static final double PEAK_VOLTAGE = 11; // Peak output of X volts on a Falcon.

        public static final double PEAK_AMPERAGE = 80; // Peak output of X amps on a Falcon.
    }

    public static class PIDConfig {
        public final double P;
        public final double I;
        public final double D;

        public final double tolerance;

        public PIDConfig(double kP, double kI, double kD, double tolerance) {
            this.P = kP;
            this.I = kI;
            this.D = kD;
            this.tolerance = tolerance;
        }

        public PIDConfig(double kP, double kI, double kD) {
            this(kP, kI, kD, 0.0);
        }

        public PIDController toPIDController() {
            var controller = new PIDController(P, I, D);
            controller.setTolerance(tolerance);
            return controller;
        }
    }

    public static final class DriveTrain {
        public static final Translation2d LOCATION_FRONT_LEFT = new Translation2d(0.238, 0.238);
        public static final Translation2d LOCATION_FRONT_RIGHT = new Translation2d(0.238, -0.238);
        public static final Translation2d LOCATION_BACK_LEFT = new Translation2d(-0.238, 0.238);
        public static final Translation2d LOCATION_BACK_RIGHT = new Translation2d(-0.238, -0.238);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                LOCATION_FRONT_LEFT, LOCATION_FRONT_RIGHT, LOCATION_BACK_LEFT, LOCATION_BACK_RIGHT);

        public static final PIDConfig PID_X = new PIDConfig(0.05, 0, 0, 3);
        public static final PIDConfig PID_Y = new PIDConfig(0.05, 0, 0, 3);
        public static final PIDConfig PID_T = new PIDConfig(1.0, 0.1, 0.05, Math.toRadians(8));
    }
}
