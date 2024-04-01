package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Global_Variables
{
    public static boolean left_trigger_boost = false;
    public static boolean right_trigger_boost = false;
    public static boolean isIntaking = false;

    public static double yaw = 0;
    public static double pitch = 0;
    public static double roll = 0;

    public static double tx = 0;
    public static double ty = 0;
    public static double tv = 0;
    public static double distanceY = 0;
    public static double distanceYFixed = 0;
    public static double distanceX = 0;
    // public static final SendableChooser<Boolean> pipeline_chooser = new SendableChooser<>();


    public static double robot_directionY = 1;
    public static boolean isShooting = false;

    public static double yawFixed = 0.0;
    public static double swerveLimelightTarget = 0;

    public static double turretPos = 0;
    public static double turretTarget = 0;

    public static double shooterTarget = 0;
    public static double shooterSpeed = 0;


    
    private static DigitalInput noteSensor = new DigitalInput(9);


    public static int getSensorVal(){
      if(noteSensor.get()){
        return 1;
      }
      else{
        return -1;
      }
    }

    /**@returns true: if limelight tx is lined up with apriltag
     * <li> false: if limelight tx is not lined up with apriltag
     */
    public static boolean isSwerveAimed(){
      if(Global_Variables.tx < 3.0 && Global_Variables.tx > -3.0){
        return true;
      }
      else{
        return false;
      }
    }
    
    public static boolean auton_Timer(int timeElapsed, int time){
      if(timeElapsed<time){
        timeElapsed++;
        return false;
      }
      else{
        return true;
      }
    }
    
}