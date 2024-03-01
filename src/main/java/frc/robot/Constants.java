package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final String CANIVORE_NAME = "CANivore";
    public static final double DRIVE_SPEED = 0.6;


    public static final double ARM_OFFSET = 92.0; //182.0/360;


    public static final double ARM_GEAR_RATIO = 268.6815;   //  22.67*(64/18)*(60/15)
    public static final double ARM_CONVERSION = 2048*ARM_GEAR_RATIO/360; // Degrees*ARM_CONVERSION = Encoder value
    public static final double ARM_CONVERSION2 = 1*ARM_GEAR_RATIO/360; // Degrees*ARM_CONVERSION = Rotations
    public static final double APRIL_TAG_HEIGHT = 57;// 57.25; //54
    public static final double LIMELIGHT_HEIGHT = 14;// 57.25; //54
    public static final double LIMELIGHT_ANGLE = 32;// 57.25; //54
    public static final double SHOOTER_OFFSET_Y = 0; // Distance from Limelight
    public static final double SHOOTER_OFFSET_X = 0; // Distance from Limelight

    public static final double SPEAKER_HEIGHT = 92.193;


    public static final double Mod0_ROTATION_OFFSET = (-147.12 + 180)%360;
    public static final double Mod1_ROTATION_OFFSET = (113.38 + 180)%360;
    public static final double Mod2_ROTATION_OFFSET = (-161.36 + 180)%360;
    public static final double Mod3_ROTATION_OFFSET = (79.80 + 180)%360;


    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot90
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(20); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double driveBaseRadius = Units.inchesToMeters(13.75);

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;//0.12 //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (0.667 / 12);//0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);//1.51;
        public static final double driveKA = (0.27 / 12);//0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20; // 20
            public static final int angleMotorID = 30; //30
            public static final int canCoderID = 10; //10
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod0_ROTATION_OFFSET); //210.36
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21; //21
            public static final int angleMotorID = 31; // 31
            public static final int canCoderID = 11; //11
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod1_ROTATION_OFFSET);//133.8
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 22; //22
            public static final int angleMotorID = 32; // 32
            public static final int canCoderID = 12; //12
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod2_ROTATION_OFFSET);//13.27
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 
        { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 23; //23
            public static final int angleMotorID = 33; //33
            public static final int canCoderID = 13; //13
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod3_ROTATION_OFFSET); //357.4
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.6; //20;  //1.3
        public static final double kPYController = 1.3;  //5; //1.3
        public static final double kPThetaController = 10;//-20  //-10;;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    
    public static final int Arm_Motor_Slave = 46;
    public static final int Arm_Motor = 45;
    public static final int Arm_CANCoder = 40;

}   
