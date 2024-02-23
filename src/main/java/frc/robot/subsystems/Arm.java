// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
/*Assignments:
 * Add deaband to the field centric arm so that there is less of a problem around 180 yaw
 * add constant for one side of the robot
*/
public class Arm extends SubsystemBase 
{

  Orchestra _orchestra;

  private final TalonFX _arm = new TalonFX(Constants.Arm_Motor, Constants.CANIVORE_NAME );
  private final TalonFX _arm2 = new TalonFX(Constants.Arm_Motor_Slave, Constants.CANIVORE_NAME );
  private CANcoder armCANCoder = new CANcoder(Constants.Arm_CANCoder, Constants.CANIVORE_NAME);
  
  private MotionMagicDutyCycle m = new MotionMagicDutyCycle(0);   
private VoltageOut e = new VoltageOut(0, true, false, false, false);

  private static TalonFX[] _instruments = new TalonFX[1];
  

  int _timeToPlayLoops = 0;
  /** Creates a new Arm. */
  public void arm_init()
  {
    _arm.setNeutralMode(NeutralModeValue.Brake); //Brake
    _arm2.setNeutralMode(NeutralModeValue.Brake); //Brake
    _arm.setInverted(false);
    if(getArm_CANCoder()!=0)
    {
        _arm.getConfigurator().setPosition((Constants.ARM_OFFSET-armCANCoder.getAbsolutePosition().getValueAsDouble())*Constants.ARM_CONVERSION);
   }
    else
    {
        _arm.getConfigurator().setPosition(0);
    }

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 20000; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 20000; //80 // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 4;// Take approximately 0.2 seconds to reach max accel 

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 12.011730205278592; //5
    slot0.kI = 0.48046920821114375;
    slot0.kD = 0.0; //0.1
    slot0.kV = 0.0; //0.12
    slot0.kS = 0.0; //0.25 // Approximately 0.25V to get the mechanism moving

    _arm.getConfigurator().apply(cfg, 0.050);

    FeedbackConfigs fConfigs = cfg.Feedback;
    fConfigs.SensorToMechanismRatio = Constants.ARM_GEAR_RATIO;



    // _arm.configMotionCruiseVelocity(20000);
    // _arm.configMotionAcceleration(20000);
    // _arm.configMotionSCurveStrength(4); //4

    // _arm.configForwardSoftLimitEnable(false);
    // _arm.configForwardSoftLimitThreshold(0);
    _arm2.setControl(new Follower(Constants.Arm_Motor, false));
}

  public void music_init()
  {
  //   ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
  //   _instruments.add(_arm);
  //   _instruments.add(_arm2);
  //   _orchestra = new Orchestra(_instruments);
  //  // _orchestra.loadMusic("DMX.chrp");
  //   _orchestra.loadMusic(Global_Variables.song.getSelected());
  //   _timeToPlayLoops = 10;
  _instruments[0] = _arm; //
  _instruments[1] = _arm2;
  }

  public void play_music()
  {
    if (_timeToPlayLoops > 0) {
      --_timeToPlayLoops;
      if (_timeToPlayLoops == 0) {
          /* scheduled play request */
          System.out.println("Auto-playing song.");
          _orchestra.play();
      }
  }
  }

  public TalonFX[] returnArmMotors()
  {
    _instruments[0] = _arm; //
    _instruments[1] = _arm2;  
    return _instruments;
  }

 /*Button Control :) */
 public void arm_on(double speed)
 {
   _arm.setControl(e.withOutput(Global_Variables.robot_direction*speed*12));
 }

 public void arm_resetEncoder()
 {
    _arm.getConfigurator().setPosition(0);
 }


 public void Arm_Goto_Angle(double angle)
 {
  _arm.setControl(m.withPosition(angle));
 }


 public void arm_off()
  {
   _arm.setControl(e.withOutput(0));
  }

  public double getArm_encoder()
  {
    // double arm_encoder = _arm.getSelectedSensorPosition()/Constants.ARM_CONVERSION;
    double arm_encoder = Rotation2d.fromRotations(_arm.getPosition().getValue()).getDegrees();///Constants.ARM_CONVERSION;
    return arm_encoder;
  }

  public double getArm_CANCoder()
  {
    double arm_CANCoder = armCANCoder.getPosition().getValue();//;.getAbsolutePosition();
    return arm_CANCoder;
  }

  public double Arm_Speed() 
  {
    return (_arm.getVelocity().getValue()+_arm2.getVelocity().getValue())/2;
  }

  public void updateEncoder()
{
  if(getArm_CANCoder()!=0)
  {
    // _arm.setSelectedSensorPosition((Constants.ARM_OFFSET-armCANCoder.getAbsolutePosition())*Constants.ARM_CONVERSION);  //  angle*2048*100(gear ratio)/360
    _arm.getConfigurator().setPosition(Constants.ARM_OFFSET-Rotation2d.fromDegrees(armCANCoder.getAbsolutePosition().getValueAsDouble()).getDegrees());//*Constants.ARM_CONVERSION));

  }
}



;
  public int getAllianceColor()
  {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Blue) {
            return -1;
        }
        if (ally.get() == Alliance.Red) {
            return 1;
        }
        else{
            return 0;
        }
    }
    else{
        return 0;
    }
  }
  
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Right Bumper", Global_Variables.right);
    // SmartDashboard.putBoolean("Left Bumper", Global_Variables.left_bumper);
    SmartDashboard.putNumber("Arm Speed", Arm_Speed());

    SmartDashboard.putNumber("Arm Encoder", getArm_encoder());
    SmartDashboard.putNumber("Arm CANCoder", getArm_CANCoder());
  }
}