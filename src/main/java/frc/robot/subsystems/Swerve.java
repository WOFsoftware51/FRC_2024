package frc.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public double speedMod  = Constants.DRIVE_SPEED;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public double tx = 0.0;
    public double ty = 0.0;
    public double tv = 0.0;
    public double distanceY = 0.0;
    public double distanceX = 0.0;
    public double distanceXY = 0.0;
    public double txCenterRobot = 0.0;  
    public double yawFixed = 0.0;

    Optional<Alliance> ally = DriverStation.getAlliance();

    public double botpose[];


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);
        // Pigeon2Configuration gyroConfigs = new Pigeon2Configuration();
        // gyroConfigs.Pigeon2Features.withEnableCompass(false);
        // gyro.getConfigurator().apply(gyroConfigs);

        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0, 0.1);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());//, new Pose2d()); //TODO Test This

        configureAuton();
    }
    /**Drive command used in auton */
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
    /** Robot Relative Drive
     * <P>Drive command used in auton */
    public void driveRelative(ChassisSpeeds driveMSupplier) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(driveMSupplier);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true); //false); //TODO test this
        }
        //Use this instead?
        /*setModuleStates(swerveModuleStates) */
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
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates());
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
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
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
        gyro.setYaw(0, 0.1);
    }
    public void setGyro(double newValue){
        gyro.setYaw(newValue, 0.1);
    }

    public void setGyro60(){
        gyro.setYaw(60, 0.1);
    }
    public void setGyroN60(){
        gyro.setYaw(-60, 0.1);
    }
    /**Zeroes both the Gyro and the Heading */
    public void zeroDrive(){
        zeroGyro();
        zeroHeading();
    }
    /**Sets the gyro and the heading */
    public void setDrive(double newGyro, double newHeading){
        setGyro(newGyro);
        setHeading(new Rotation2d(Math.toRadians(newHeading)));
    }

    public void flipHeading(){
        setHeading(new Rotation2d(getHeading().getRadians()).unaryMinus());
    }

    public void boostOn(){
        speedMod = 1.0;
    }
     public void boostOff(){
        speedMod = Constants.DRIVE_SPEED;
    }
        
    private double rotationVal = 0;
    public double gotoDefaultGyroVal(){
        if(yawFixed>180){
            rotationVal = 0.6;
        }
        else if(yawFixed<180){
            rotationVal = -0.6;
        }
        if(yawFixed>345){
            rotationVal = 0.1;
        }
        else if(yawFixed<15){
            rotationVal = -0.1;
        }

        if(yawFixed > 357 || yawFixed < 3)
        {
            rotationVal = 0;
        }
        return rotationVal;
    }
    /**Aiming Swerve with Limelight Towards AprilTag */
    public double limelight_aim_proportional()
    {    
        double targetingAngularVelocity = 0;
        final double kP = 0.001666/2;
        final double kI = 0.000002*8*2*16*16*2 ; // 0.016;
        final double kD = 0.0; // 0.00000125;
        PIDController AimPID = new PIDController(kP, kI, kD);


        targetingAngularVelocity = AimPID.calculate(limelightTarget(), 0);// -(tx * kP + kD*txRateOfChange() + kI*txIntegral());
        
        targetingAngularVelocity *= 6;
  
        targetingAngularVelocity *= 1;

        return targetingAngularVelocity;
    }

    public double limelightTarget(){
        return Global_Variables.tx;
    }

    /**Directly follows a Pathplanner path
     * <p> NOTE: May not work because pose get weird. Sidestepped the issue by following a Pathplanner Auton and overriding the Pose in there instead.
     */
    public Command followTrajectoryCommand(String pathString, boolean isFirstPath) {
    
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
    
        return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(  // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    Constants.Swerve.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () ->  false, //Probably doesn't need to flip because field lacks symetry
                      // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //     return alliance.get() == DriverStation.Alliance.Red;
                // }
                //     return false;
                // 
            this // Reference to this subsystem to set requirements
        );
    }

    /**Configurer for Pathplanner Auton */
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
                new ReplanningConfig()), // Default path replanning config. See the API for the options here
            // () -> false,
            ()-> false,
            this // Reference to this subsystem to set requirements
        );
    }
    /**Resets Yaw
     *<p>Values range from [-180, 180]
     */
    public void setYawWrapped(double newAngle){
        gyro.setYaw(newAngle); //180-newAngle
    }

    public double getDistanceYFixed(){
        return ((0.0)*Math.pow(distanceY, 0) + 0.0);
    }

    public PathPlannerAuto pathPlannerAuto(String path){

        return new PathPlannerAuto(path);
    }
    @Override
    public void periodic(){
        // swerveOdometry.update(getGyroYaw(), getModulePositions());

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        yawFixed = Math.abs((360-gyro.getAngle())% 360);

        SmartDashboard.putNumber("yawFixeds", yawFixed); 
        SmartDashboard.putNumber("yaw", getGyroYaw().getDegrees()); 


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        tv = table.getEntry("tv").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        // botpose = table.getEntry("botpose").getDoubleArray(new double[6]); 
        distanceY = (Constants.APRIL_TAG_HEIGHT-Constants.LIMELIGHT_HEIGHT)/(Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE+ty)));
        distanceX = distanceY/(Math.tan(Math.toRadians(tx)));


        // txCenterRobot = Math.atan(distanceY/(distanceX+Math.tan(Global_Variables.tx)));
        // // // // double bLat = botpose[6];

        SmartDashboard.putNumber("tx", Global_Variables.tx);
        SmartDashboard.putNumber("tv", Global_Variables.tv);
        SmartDashboard.putNumber("ty", Global_Variables.ty);

        SmartDashboard.putNumber("Drive Speed Chassis Speeds", Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2)));
        SmartDashboard.putNumber("Pose2d Rotation", swerveOdometry.getPoseMeters().getRotation().getDegrees());
        // if (ally.isPresent()) {
        //     if (ally.get() == Alliance.Red) {
        //         table.getEntry("pipeline").setNumber(0);
        //     }
        //     else if (ally.get() == Alliance.Blue) {
        //         table.getEntry("pipeline").setNumber(1);
        //     }
        //     // else if(Global_Variables.pipeline_chooser){
        //     //     table.getEntry("pipeline").setNumber(3);
        //     // }
        // }


        Global_Variables.tx = tx;
        Global_Variables.ty = ty;
        Global_Variables.tv = tv;
        Global_Variables.distanceY = distanceY;
        Global_Variables.distanceYFixed = getDistanceYFixed();
        Global_Variables.yawFixed = yawFixed;
        Global_Variables.swerveLimelightTarget = limelight_aim_proportional();
    }  
}