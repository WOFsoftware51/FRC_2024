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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Swerve extends SubsystemBase {
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;
    public double speedMod  = Constants.DRIVE_SPEED;

    public double yawFixed = 0.0;

    private Optional<Alliance> ally = DriverStation.getAlliance();

    private Vector<N3> driveTrainStandardDeviation = VecBuilder.fill(0, 0, 0);
    private Vector<N3> visionStandardDeviation = VecBuilder.fill(0.7,0.7,9999999);
    
    private SwerveDrivePoseEstimator m_robotPose;


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);

        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0, 0.1);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        m_robotPose =  new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d(), driveTrainStandardDeviation,visionStandardDeviation);

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
    private void driveRelative(ChassisSpeeds driveMSupplier) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(driveMSupplier);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
        //Use this instead?
        /*setModuleStates(swerveModuleStates) */
    }    

    /* Used by SwerveControllerCommand in Auto */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
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
        return m_robotPose.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_robotPose.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_robotPose.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
}

    public void zeroHeading(){
        m_robotPose.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void zeroHeadingConsumer(Pose2d heading){
        
        m_robotPose.resetPosition(getGyroYaw(), getModulePositions(), heading);
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

    /**Rotates swerve until pointing towards target angle */
    public double gotoAngle(double targetAngleDegrees){
        double kP = 0.01666;
        double kI = 0.00002;
        double kD = 0.0000125;
            
        PIDController AimPID = new PIDController(kP, kI, kD);

        double targetingAngularVelocity = AimPID.calculate(m_robotPose.getEstimatedPosition().getRotation().getDegrees(), targetAngleDegrees);
        
        targetingAngularVelocity *= 6;
  
        targetingAngularVelocity *= 1;

        return targetingAngularVelocity;
    }
    


    /**Aiming Swerve with Limelight Towards AprilTag */
    public double limelight_aim_proportional()
    {    
 
        double kP = 0.01666;
        double kI = 0.00002;
        double kD = 0.0000125;

        PIDController AimPID = new PIDController(kP, kI, kD);

        double targetingAngularVelocity = AimPID.calculate(Global_Variables.tx, 0);// -(tx * kP + kD*txRateOfChange() + kI*txIntegral());
        
        targetingAngularVelocity *= 6;
  
        targetingAngularVelocity *= 1;

        return targetingAngularVelocity;


        // double kP1 = 0.035;
        // targetingAngularVelocity = limelightTarget()*kP1;
        // targetingAngularVelocity *= 1;
        // return targetingAngularVelocity;
    }

    /**Directly follows a Pathplanner path*/
    public Command followTrajectoryCommand(String pathString) {
    
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
            () ->  false,
            this // Reference to this subsystem to set requirementsd
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


	private void addVisionToPoseEstimator(){
        PoseEstimate poseEstimator2d = Global_Variables.visionPoseEstimate2d;
        boolean rejectVision = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_robotPose.getEstimatedPosition().getRotation().getDegrees(),
            gyro.getAngularVelocityZDevice().getValue(), 
            getGyroPitch().getDegrees(), 
            gyro.getAngularVelocityXDevice().getValue(), 
            getGyroRoll().getDegrees(), 
            gyro.getAngularVelocityYDevice().getValue());

        if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectVision = true;
        }
        if(poseEstimator2d.tagCount == 0)
        {
            rejectVision = true;
        }

        if(!rejectVision){
            m_robotPose.setVisionMeasurementStdDevs(visionStandardDeviation);
            m_robotPose.addVisionMeasurement(poseEstimator2d.pose, 0.02);
        }
    }

    public PathPlannerAuto pathPlannerAuto(String path){

        return new PathPlannerAuto(path);
    }
    @Override
    public void periodic(){

        m_robotPose.update(getGyroYaw(), getModulePositions());
        addVisionToPoseEstimator();

        yawFixed = Math.abs((360-gyro.getAngle())% 360);

        SmartDashboard.putNumber("yawFixeds", yawFixed); 
        SmartDashboard.putNumber("yaw", getGyroYaw().getDegrees()); 


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }


        SmartDashboard.putNumber("Drive Speed Chassis Speeds", Math.hypot(getChassisSpeeds().vxMetersPerSecond,getChassisSpeeds().vyMetersPerSecond));

        SmartDashboard.putNumber("Pose2d X", m_robotPose.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose2d Y", m_robotPose.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pose2d Rotation", m_robotPose.getEstimatedPosition().getRotation().getDegrees());


        Global_Variables.yawFixed = yawFixed;
    }  
}