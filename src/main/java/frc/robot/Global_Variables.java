package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Global_Variables
{
  public static boolean left_trigger_boost = false;
  public static boolean right_trigger_boost = false;
  public static double robot_direction = 1;

  public static double yaw = 0;
  public static double pitch = 0;
  public static double roll = 0;

  public static double tx = 0;
  public static double ty = 0;

  public static double robot_directionY = 1;
  public static boolean isShooting = false;

  
  private static DigitalInput noteSensor = new DigitalInput(0);


  public static int getSensorVal(){
      if(noteSensor.get()){
        return 1;
      }
      else{
        return -1;
      }
    }

  public static SendableChooser<String> song = new SendableChooser<>();
    
}