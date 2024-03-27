package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final String CANIVORE_NAME = "CANivore";
    public static final double DRIVE_SPEED = 0.6;

    
    public static final double APRIL_TAG_HEIGHT = 57;// 57.25; //54
    public static final double LIMELIGHT_HEIGHT = 7.0;// 57.25; //54
    public static final double LIMELIGHT_ANGLE = 29;// 57.25; //54




    public static final double SPEAKER_HEIGHT = 92.193;


    public static final double ELEVATOR_DEFAULT = 0.0; 
    public static final double ELEVATOR_AMP = -79.322266; 
    public static final double ELEVATOR_FLOOR = 0.0; 
    public static final double ELEVATOR_TRAP = 0.0; 
    public static final double Elevator_Spool = 1.25; 
    public static final double Elevator_Gear_Ratio = 48/50; 
    public static final double Elevator_Highest_Point = -83.975098; 

    public static final double Mod0_ROTATION_OFFSET = (-134.56);
    public static final double Mod1_ROTATION_OFFSET = (-2.63);
    public static final double Mod2_ROTATION_OFFSET = (-59.68);
    public static final double Mod3_ROTATION_OFFSET = (-27.77);


    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot90
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

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
        public static final double driveKP = 0.12;//0.05;//0.12 //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;//(0.667 / 12);//0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;//(1.51 / 12);//1.51;
        public static final double driveKA = 0.27;//(0.27 / 12);//0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.0; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10;//17.5; //TODO: This must be tuned to specific robot

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
        public static final double kMaxSpxeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 3.2;//1.6; //20;  //1.3
        // public static final double kPYController = 12.8;  //5; //1.3
        public static final double kPThetaController = 7.5;//10;//-20  //-10;;
    
        public static final double shooterCurrentSpike = 32;//-20  //-10;;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


    }

    public static final class AutonTurretPositions{
        public static final class Top{
            public static final double Position_Start = 0;
            public static final double Position_1 = 0;
            public static final double Position_2 = 0;
            public static final double Position_3 = 0;

        }
        public static final class Middle{
            public static final double Position_Start = 0;
            public static final double Position_1 = 0;
            public static final double Position_2 = 0;
            public static final double Position_3 = 0;

        }
        public static final class Bottom{
            public static final double Position_Start = 0;
            public static final double Position_1 = 0;
            public static final double Position_2 = 0;
            public static final double Position_3 = 0;

        }
        public static final class Misc{
            public static final double POSITION_LEAVE_ZONE_0 = 0;
        }
    }
    public static final class Turret{

        public static final double TURRET_GEAR_RATIO = 320.0;//320; 
        public static final double TURRET_CANCODER_GEAR_RATIO = 18.0/30.0; //182.0/360; 
        public static final double TURRET_CANCODER_OFFSET = 97.45; //182.0/360; 
        public static final double TURRET_DEFAULT_POSITION = 0; //182.0/360; 
        public static final double TURRET_PROTECTED_POSITION = 23;//TURRET_DEFAULT_POSITION + 26.598999 + 6.5 ; //182.0/360; 

        /**Difference between Turret angle and the angle the note is shot */
        public static final double TURRET_SHOT_ANGLE_OFFSET = 0; //182.0/360; 
        public static final double TURRET_HEIGHT = 6.65; //182.0/360; 
        /** The height offset in comparison to the Limelight */
        public static final double TURRET_OFFSET_Y = TURRET_HEIGHT-LIMELIGHT_HEIGHT; // Distance from Limelight 
        /** The distance offset in comparison to the Limelight */ 
        public static final double TURRET_OFFSET_X = 12; // Distance from Limelight 
        /** The offset for the plane parallel to the limegight face and perpindicular to the floor*/ 
        public static final double TURRET_OFFSET_Z = 5.25; // Distance from Limelight 
        public static final double TURRET_DEFAULT_ANGLE = 65; // Distance from Limelight 




        public static final int turret = 43;//34
        public static final int turret_CANCoder = 51;
    }

    /**In RPM */
    public static final class ShooterSpeeds{
        public static final double SHOOTER_AUTON_SPEED1 = 4000;
        public static final double SHOOTER_DEFAULT_SPEED = 0;
    }

    public static final int shooter = 34;
    public static final int shooter2 = 45;

    public static final int intake = 48;
    public static final int turret = 43;//34
    public static final int turret_CANCoder = 51;


    public static final int hanger = 35;
    public static final int hanger2 = 47;

    public static final int elevator = 60;
    public static final int elevator2 = 61;
    public static final int transfer_intake = 62;
    public static final int transfer_shooter = 59;


	public static final double shootP = 0.80;//0.11
	public static final double shootI= 0.0;
	public static final double shootD= 0.0;
	public static final double shootV= 0.1033;//0.112;
	public static final double shootS= 0.05;//6;

    public static final double shoot2P = 0.80;//0.11
	public static final double shoot2I= 0.0;
	public static final double shoot2D= 0.0;
	public static final double shoot2V= 0.1033;//0.112;
	public static final double shoot2S= 0.05;//6;


    public static final int A_Button = 1;
    public static final int B_Button = 2;
    public static final int X_Button = 3;
    public static final int Y_Button = 4;

}   
