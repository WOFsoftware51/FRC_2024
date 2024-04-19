package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Global_Variables;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Turret extends SubsystemBase {

  private final TalonFX _turret = new TalonFX(Constants.turret, Constants.CANIVORE_NAME); //turret);
  private final CANcoder turretCANCoder = new CANcoder(Constants.turret_CANCoder, Constants.CANIVORE_NAME);
  private MotionMagicDutyCycle mMDutyCycle = new MotionMagicDutyCycle(0);
  private double m_target = 0;


  // private AnalogInput ns = new AnalogInput(0);

  public Turret(){

    TalonFXConfiguration cfg = new TalonFXConfiguration(); 
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 600; //400// 5 rotations per second cruise
    mm.MotionMagicAcceleration = 300; //160 // Take approximately 0.5 seconds %to reach max vel
    mm.MotionMagicJerk = 2400;//1600// Take approximately 0.2 seconds to reach max accel 


    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =  -8.6;


    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 1.92;
    slot0.kI = 0;
    slot0.kD = 0.0;
    slot0.kV = 0.0;
    slot0.kS = 0.375; // Approximately 0.25V to get the mechanism moving

    _turret.getConfigurator().apply(cfg, 0.050);

    _turret.setNeutralMode(NeutralModeValue.Brake);
    _turret.setInverted(false);


  }

  public void turret_init() 
  {
    if(getTurret_CANCoder()!=0)
    {
      updateEncoder();
    }
    else{
      _turret.getConfigurator().setPosition(0);
    }
  }

  /**Turns the turret on at a some percent speed. turret moves at x*0.5 speed 
   * @param x
   * <ul> 
   *  <li> Units: Percantage Range from [-1, 1]. 
   *  <li> x is multiplied by 0.5 in order slow its speed down
  */
  public void turretOn(double x){
    _turret.set(x*0.5); //0.3
  }
  /**
   * Rotates the turret to some angle in degrees.
   * <li> This angle is based on the turret motor times the turret's gear ratio. This gives us the turret's actual physical angle 
   * @param target
   * <ul> 
   *  <li> Units: Degrees.
  */
  public void turret_Goto_angle(double target){
    m_target = target;
    mMDutyCycle.Slot = 0;
    _turret.setControl(mMDutyCycle.withPosition(((target)/360)*Constants.Turret.TURRET_GEAR_RATIO)); //Divide by 360 to convert Degrees to Rotations
  }
  
  public void turretOff(){
    // _turret.setControl(ControlModeValue.,x);
      _turret.set(0);
  }

  public void resetSensorPos(){
    _turret.setPosition(0);
  }

  /** In Degrees*/
  public double getTurretEncoder(){
    return _turret.getRotorPosition().getValue()*360/Constants.Turret.TURRET_GEAR_RATIO;
  }

  public double getTurret_CANCoder(){
    double turret_CANCoder = turretCANCoder.getAbsolutePosition().getValue()*360*Constants.Turret.TURRET_CANCODER_GEAR_RATIO;//;.getAbsolutePosition();
    return turret_CANCoder;
  }

  public void updateEncoder(){
    if(getTurret_CANCoder()!=0){
      // _turret.getConfigurator().setPosition((Constants.Turret.TURRET_CANCODER_OFFSET-(getTurret_CANCoder()))/Constants.Turret.TURRET_GEAR_RATIO);
      _turret.getConfigurator().setPosition((getTurret_CANCoder()-Constants.Turret.TURRET_CANCODER_OFFSET)*Constants.Turret.TURRET_GEAR_RATIO/360);
    }
  }
  /**
   * @return the turret angle to score based on the limelight
   */
  public double getTurretAimTarget(){

    /*Limelight 3g Pre-Day 1 Livonia*/
    // return (((-5311800000000000.0)*(Math.pow(((0.0220706*Global_Variables.distanceY) + 30.4041), -9.308))) + 62.9833);
    
    /*Limelight 3g Post-Day 1 Livonia*/
    return (((-2773100000000000.0)*(Math.pow(((0.000511024*Global_Variables.distanceY) + 1.88409), -49.4666))) + 37.7033);

    /*Limelight 2 */
    // return (((-5338700000000000.0)*(Math.pow(((0.001158444*Global_Variables.distanceY) + 3.45686), -25.823))) + 44.1208);

    /*Linear Equation using Limelight 2*/
      // return (0.211991*Global_Variables.distanceY) - 6.77857;

  } 

  @Override
  public void periodic(){ 
    SmartDashboard.putNumber("Turret Angle", getTurretEncoder());
    SmartDashboard.putNumber("Turret CANCoder Angle", getTurret_CANCoder());

    SmartDashboard.putNumber("Offset Math", (getTurret_CANCoder()-Constants.Turret.TURRET_CANCODER_OFFSET)/Constants.Turret.TURRET_GEAR_RATIO/360);

    SmartDashboard.putBoolean("Turret Forward Limit", _turret.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
    SmartDashboard.putBoolean("Turret Rev Limit", _turret.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
    SmartDashboard.putNumber("getTurretAimTarget", getTurretAimTarget());
    SmartDashboard.putNumber("Turret Real Target", m_target);
    SmartDashboard.putNumber("Turret Cancoder Pos", turretCANCoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Turret Pos", getTurretEncoder()*Constants.Turret.TURRET_GEAR_RATIO/360);
    SmartDashboard.putNumber("distanceY", Global_Variables.distanceY);
    SmartDashboard.putNumber("Turret Math Power", -5.3384*1000000000000000.0*(Math.pow((-0.00549732*Global_Variables.distanceY) + 9.13569, -14.4726)));


    // turretAngleToScore = Math.atan((Constants.SPEAKER_HEIGHT - Constants.Turret.TURRET_HEIGHT)/(Global_Variables.distanceY+Constants.Turret.TURRET_OFFSET_X)) * 180/(Math.PI); 
    Global_Variables.turretPos = getTurretEncoder();
    Global_Variables.turretTarget = m_target;
  }

  } 