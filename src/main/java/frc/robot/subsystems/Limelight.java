// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public double tx = 0.0;
    public double ty = 0.0;
    public double tv = 0.0;
    public double distanceY = 0.0;
    public double distanceX = 0.0;
    public double distanceXY = 0.0;
    public double txCenterRobot = 0.0;  
    public double yawFixed = 0.0;

    private Pose2d startingPose2d;
    private Pose2d visionPoseStartClone;
    /** Creates a new Limelight. */
    public Limelight() {
        startingPose2d = new Pose2d();
        visionPoseStartClone = getVisionPoseEstimate2d().pose;
    }
    private double getDistanceYFixed(){
        return ((0.0)*Math.pow(distanceY, 0) + 0.0);
    }
    private PoseEstimate getVisionPoseEstimate2d(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }
    private Pose2d updateStartingPose2d(){
        Transform2d t = getVisionPoseEstimate2d().pose.minus(visionPoseStartClone);
        return startingPose2d = new Pose2d(t.getTranslation(), t.getRotation());
    }
    

  @Override
  public void periodic() {
    tv = table.getEntry("tv").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    tx = table.getEntry("tx").getDouble(0);
    distanceY = (Constants.APRIL_TAG_HEIGHT-Constants.LIMELIGHT_HEIGHT)/(Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE+ty)));
    distanceX = distanceY/(Math.tan(Math.toRadians(tx)));
    Global_Variables.tx = tx;
    Global_Variables.ty = ty;
    Global_Variables.tv = tv;
    Global_Variables.distanceY = distanceY;
    Global_Variables.distanceYFixed = getDistanceYFixed();
    Global_Variables.visionPoseEstimate2d = getVisionPoseEstimate2d();
    Global_Variables.visionPoseStart = updateStartingPose2d();

    SmartDashboard.putNumber("tx", Global_Variables.tx);
    SmartDashboard.putNumber("tv", Global_Variables.tv);
    SmartDashboard.putNumber("ty", Global_Variables.ty);
    SmartDashboard.putNumber("Vision Pose X", Global_Variables.visionPoseEstimate2d.pose.getX());
    SmartDashboard.putNumber("Vision Pose Y", Global_Variables.visionPoseEstimate2d.pose.getY());

    // SmartDashboard.putNumber("Vision Pose X From Start ", Global_Variables.visionPoseStart.getX());
    // SmartDashboard.putNumber("Vision Pose Y Fromt Start", Global_Variables.visionPoseStart.getY());





}
}
