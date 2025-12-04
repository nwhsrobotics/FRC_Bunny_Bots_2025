package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public final class Constants {


    public static final class CANAssignments {
        // SCORING SUBSYSTEM MOTOR ID
        public static final int PIVOT_ARM_MOTOR_ID = 0;
        public static final int INTAKE_MOTOR_ID = 0;

        public static final int INDEX_MOTOR_ID = 0;
        
        public static final int FLYWHEEL_ONE_ID = 0;
        public static final int FLYWHEEL_TWO_ID = 0;
        
        public static final int FEEDER_MOTOR_ID = 0;

        // SWERVE MOTOR ID
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 27;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 6;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 9;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 12;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 10;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 11;

        public static final int PDU_ID = 24;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10033; // set up for MK4(i)
        public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // (set up for MK4(i) L2)
        public static final double kTurningMotorGearRatio = (15.0 / 32.0) * (10.0 / 60.0); // (set up for MK4 L2)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .5; // P constant for turning
        //public static final double kPTolerance = 2.5 * (Math.PI/180);
        public static final double kITurning = 0.;
    }

    public final class Buttons {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int VIEW = 7;
        public static final int MENU = 8;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;
        public static final int POV_UP = 0;
        public static final int POV_DOWN = 180;
        public static final int POV_RIGHT = 90;
        public static final int POV_LEFT = 270;
    }

    public static final class DriveConstants {
        // left-to-right distance between the drivetrain wheels, should be measured from center to center AND IN METERS
        public static final double kTrackWidth = 0.5715;
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.5715;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //back right

        //0.9, 1.35, 1.5
        public static final double kDirectionSlewRate = 0.95; // radians per second
        public static final double kMagnitudeSlewRate = 1.425; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 1.3; // percent per second (1 = 100%)

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoders
        //THE BLACK GEAR SHOULD BE ON THE OUTSIDE FOR ALL WHEELS, regardless of side


        //THESE ONES ARE FOR THE 2025 ROBOT 
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.607456392002714;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.6902913545485385;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.06934008285773;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.2779614700101773;//5.20 - Math.PI;


        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS
    }


    public static final class AutoConstants {
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kPXController = 5;
        public static final double kPThetaController = 5;

        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
                new PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants
                new PIDConstants(AutoConstants.kPThetaController, 0, 0)// Rotation constants
        );

        public static final PathConstraints kPathfindingConstraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.3,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double scaleFactor = 0.6;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class Positions {

        public static final Pose2d BLUE_REEF_CENTER = new Pose2d(4.5, 4.05, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_REEF_CENTER = new Pose2d(13, 4.05, Rotation2d.fromDegrees(0));

        public static final List<Pose2d> REEF_CENTERS = new ArrayList<>(List.of(BLUE_REEF_CENTER, RED_REEF_CENTER));
    }

    public static final class ScoringConstants {
        public static final double PIVOT_GEAR_RATIO = 52;

        public static final double INTAKE_SPEED = 4500; //Need to tune these big time
        public static final double FEEDER_SPEED = 0.7; //Need to tune these big time
        public static final double FEEDER_CLEAR_SPEED = 0.6; //Need to tune these big time
        public static final double LOW_SHOT_RPM = 3500.0;  // tune me
        public static final double HIGH_SHOT_RPM = 6000.0; // tune me
        public static final double RPM_TOLERANCE = 150.0;  // considered "at speed"
        public static double targetRPM = 0;
        public double basePower = 0; // for feedforward

        public static double MAX_VELOCITY_RPM = 0.0; //need to tune obv
        public static double MAX_ACCEL_RPM_S = 5.0; //need to tune

        public static double OUTPUT_PER_RPM = 0.0; //need to tune
    }

    public static final class MotorConstants{

    }

    


    public static final class LimelightConstants {

        public static String llFront = "limelight-llf";
        public static String llBack = "limelight-llb";
    }

    public static final class AprilTagConstants{
        private static double m(double in) { return in * 0.0254; }

    // Tag coordinates from the 2025 manual (COSMIC CONVERTERS)
        public static final Pose2d TAG_5 = new Pose2d(m(4.0),   m(196.125), Rotation2d.fromDegrees(0));     // Blue Converter
        public static final Pose2d TAG_6 = new Pose2d(m(644.0), m(196.125), Rotation2d.fromDegrees(180));   // Red Converter
        public static final Pose2d TAG_7 = new Pose2d(m(4.0),   m(20.5),    Rotation2d.fromDegrees(0));     // Blue Converter
        public static final Pose2d TAG_8 = new Pose2d(m(644.0), m(20.5),    Rotation2d.fromDegrees(180));   // Red Converter

    /** Return pose if ID is a COSMIC CONVERTER tag; otherwise return null. */
        public static Pose2d getTagPose(int id) {
            return switch(id) {
                case 5 -> TAG_5;
                case 6 -> TAG_6;
                case 7 -> TAG_7;
                case 8 -> TAG_8;
                default -> null;   // ignore STARSPIRE tags (1–4)
            };
        }
        }

    public static final class AutoAlignConstants {
        public static final double TOUCHING_M = 0.762;   // 30 inches
        public static final double SHOOTING_M = 1.8288;  // 72 inches
    }


    public enum RuntimeEnvironment {
        /**
         * Running on physical robot.
         */
        REAL,
        /**
         * Running on simulated robot.
         */
        SIMULATION,
        /**
         * Replaying robot from log file.
         */
        REPLAY
    }

    public static final ModuleType PDU_TYPE = ModuleType.kRev;

    public static final class LoggerConstants {
        public static final RuntimeEnvironment MODE = RuntimeEnvironment.REAL;
        public static final String RUNNING_UNDER = "2025.q2";

        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}
