package frc.robot.subsystems;

import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Global_Variables;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Turret extends SubsystemBase {

  private TalonFX _turret = new TalonFX(Constants.turret); //turret);
  private MotionMagicDutyCycle mMDutyCycle = new MotionMagicDutyCycle(0);
  private VoltageOut vOut = new VoltageOut(0, false, false, false, false);

  // private AnalogInput ns = new AnalogInput(0);

  public void turret_init() 
  {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 80; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 160; //80 // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 800;// Take approximately 0.2 seconds to reach max accel 

    
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 0.5; //5
    slot0.kI = 0;
    slot0.kD = 0.0; //0.1
    slot0.kV = 0.0; //0.12
    slot0.kS = 0.0; //0.25 // Approximately 0.25V to get the mechanism moving

    _turret.getConfigurator().apply(cfg, 0.050);

    FeedbackConfigs fConfigs = cfg.Feedback;
    fConfigs.SensorToMechanismRatio = 70;

    _turret.setNeutralMode(NeutralModeValue.Brake);
    _turret.setInverted(false);
  }
  
  public void turretOn(double x){
    // _turret.setControl(d.withSlot(0));
    // _turret.setPosition(30);
    
    _turret.setControl(vOut.withOutput(12*x)); //0.3
    // _turret.setControl(e.withOutput(x*0.8));

  }


  public void turret_Goto_angle(double x){
    mMDutyCycle.Slot = 0;
    _turret.setControl(mMDutyCycle.withPosition(x*360));
  }
  
  public void turretOff(){
    // _turret.setControl(ControlModeValue.,x);
      _turret.set(0);
  }

  public void resetSensorPos(){
    _turret.setPosition(0);
  }

  public double getTurretSensorPos(){
    return _turret.getPosition().getValue();
  }


  @Override
  public void periodic(){
    SmartDashboard.putNumber("Turret Pos", getTurretSensorPos());

    SmartDashboard.putNumber("Sensor Val", Global_Variables.getSensorVal());

  }

  }