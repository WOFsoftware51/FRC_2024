package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public double speedMod = Constants.DRIVE_SPEED;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public double tx = 0.0;
    public double ty = 0.0;
    public double tv = 0.0;
    public double distance = 0.0;
    public double yawFixed = 0.0;
    public double tx1 = 0;
    public double turretAngleToScore = 0.0;


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Timer.delay(1.0);
        // resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        configureAuton();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) 
    {
        SwerveModuleState[] swerveModuleStates =    
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void driveRelative(ChassisSpeeds driveMSupplier) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                driveMSupplier
             );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates()); // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    }    

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void zeroHeadingConsumer(Pose2d heading){
        
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), heading);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }
    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll().getValue());
    }
    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void boostOn(){
        speedMod = 1.0;
    }
    public void boostOff(){
        speedMod = Constants.DRIVE_SPEED;
    }

    public double mod0DriveEncoder() {
        return mSwerveMods[0].driveEncoder();
    }

    
    public Command followTrajectoryCommand(String pathString) {
    
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
    
        return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants 
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
            Constants.Swerve.maxSpeed, // Max module speed, in m/s
            Constants.Swerve.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            0.02,
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> false, // Probably doesn't need to flip because field lacks symetry
                // {
                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //     return alliance.get() == DriverStation.Alliance.Red;
                // }
                //     return false;
                // 
            // },
            this // Reference to this subsystem to set requirements
        );
    }

    public void configureAuton(){
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::zeroHeadingConsumer, // Robot pose supplier
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants 
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                Constants.Swerve.maxSpeed, // Max module speed, in m/s
                Constants.Swerve.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, false)), // Default path replanning config. See the API for the options here
            () -> false,
            this // Reference to this subsystem to set requirements
        );
    }

    public double limelight_aim_proportional()
    {    
        double targetingAngularVelocity = 0;
        final double kP = 0.001666;
        final double kI = 0.000000; // 0.016;
        final double kD = 0.00000125; // 0.008;
        PIDController AimPID = new PIDController(kP, kI, kD);
        
        targetingAngularVelocity = AimPID.calculate(tx, 0);// -(tx * kP + kD*txRateOfChange() + kI*txIntegral());
        
        targetingAngularVelocity *= 4;
  
        targetingAngularVelocity *= 1;

        return targetingAngularVelocity;
    }

    @Override
    public void periodic(){
        // swerveOdometry.update(getGyroYaw(), getModulePositions());

        swerveOdometry.update(getGyroYaw(), getModulePositions());  
        yawFixed = Math.abs(gyro.getYaw().getValue()% 360);


        if(yawFixed < 270 && yawFixed > 90)
        {
          Global_Variables.robot_direction = -1.0;
        }
        else
        {
         Global_Variables.robot_direction = 1.0;
        }

        if(yawFixed < 180 && yawFixed > 0)
        {
          Global_Variables.robot_directionY = -1.0;
        }
        else
        {
         Global_Variables.robot_directionY = 1.0;
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("Mod0 Drive Encoder", mod0DriveEncoder());

        tv = table.getEntry("tv").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        distance = (Constants.APRIL_TAG_HEIGHT-Constants.LIMELIGHT_HEIGHT)/(Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE+ty)));
        
        SmartDashboard.putNumber("tx", Global_Variables.tx);
        SmartDashboard.putNumber("tv", tv);
        SmartDashboard.putNumber("ty", Global_Variables.ty);

        Global_Variables.yaw = getGyroYaw().getDegrees();
        Global_Variables.roll = getGyroRoll().getDegrees();
        Global_Variables.pitch = getGyroPitch().getDegrees();
        Global_Variables.tx = tx;
        Global_Variables.ty = ty;
        Global_Variables.distance = distance;

        SmartDashboard.putNumber("Distance", distance); 
        SmartDashboard.putNumber("Drive Speed Chassis Speeds", Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2)));
   
    }  
}