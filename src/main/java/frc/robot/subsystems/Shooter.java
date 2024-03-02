package frc.robot.subsystems;

import frc.robot.Constants;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {



    private TalonFX _shooter = new TalonFX(Constants.shooter);
    private TalonFX _shooter2 = new TalonFX(Constants.shooter2);
    private VelocityVoltage d = new VelocityVoltage(0,0,true,0,0,false,false,false);
    private double m_velocity = 0; 

    public void shooter_init() 
    {
      _shooter.setNeutralMode(NeutralModeValue.Brake);
      _shooter2.setNeutralMode(NeutralModeValue.Brake);
      _shooter.setInverted(true);

      TalonFXConfiguration configs = new TalonFXConfiguration();

      configs.Slot0.kP = Constants.shootP;
      configs.Slot0.kI = Constants.shootI;
      configs.Slot0.kD = Constants.shootD;
      configs.Slot0.kV = Constants.shootV;
      configs.Slot0.kS = Constants.shootS;

      configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
      configs.withDifferentialSensors(new DifferentialSensorsConfigs());
  

      _shooter.getConfigurator().apply(configs, 30);
      _shooter.setInverted(true);
      // _shooter.setStatusFramePeriod(1, 20);
  
  
      // _shooter.selectProfileSlot(0, 0);
    
      _shooter2.setControl(new Follower(Constants.shooter, true));
    }


    public void shooterOn(double velocity){
      m_velocity = velocity/60;
      _shooter.setControl(d.withVelocity(m_velocity).withFeedForward(0.5)); // Divide by 60 to go from RPM -> RPS 
    }
    public void shooterOff(){
      _shooter.set(0);
    }
    public void shooterOnTop(){
      _shooter.set(-0.16);  
    }

    public double getVelocity(){
      return _shooter.getVelocity().getValueAsDouble()*(60);
    }

    public double getTargetVelocity(){
      return m_velocity;
    }

    public double getCurrent(){
      return _shooter.getStatorCurrent().getValue();
    }
 

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter RPM", getVelocity());
      SmartDashboard.putNumber("Shooter RPS", getVelocity()/60);
      SmartDashboard.putNumber("Shooter TargetSpeed", m_velocity);
    }
}