package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Global_Variables;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {



    private TalonFX _shooter = new TalonFX(Constants.shooter, Constants.CANIVORE_NAME);
    private TalonFX _shooter2 = new TalonFX(Constants.shooter2, Constants.CANIVORE_NAME);
    private VelocityVoltage velocityController= new VelocityVoltage(0,0,true,0,0,false,false,false);
    private double m_velocity = 0; 

    public Shooter(){
      _shooter.setNeutralMode(NeutralModeValue.Coast);
      _shooter2.setNeutralMode(NeutralModeValue.Coast);
      _shooter.setInverted(false);
      _shooter2.setInverted(true);

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
      _shooter2.getConfigurator().apply(configs, 30);

      // // _shooter.setStatusFramePeriod(1, 20);
  
  
      // // _shooter.selectProfileSlot(0, 0);
    
      // // _shooter2.setControl(new Follower(Constants.shooter, true));
    }

    public void shooter_init() 
    {
      // _shooter.setNeutralMode(NeutralModeValue.Coast);
      // _shooter2.setNeutralMode(NeutralModeValue.Coast);
      // _shooter.setInverted(false);
      // _shooter2.setInverted(true);

      // TalonFXConfiguration configs = new TalonFXConfiguration();

      // configs.Slot0.kP = Constants.shootP;
      // configs.Slot0.kI = Constants.shootI;
      // configs.Slot0.kD = Constants.shootD;
      // configs.Slot0.kV = Constants.shootV;
      // configs.Slot0.kS = Constants.shootS;

      // configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      // configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
      // configs.withDifferentialSensors(new DifferentialSensorsConfigs());


      // _shooter.getConfigurator().apply(configs, 30);
      // _shooter2.getConfigurator().apply(configs, 30);
    }

    /**
     * 
     * @param velocity Velocity in Rotations per Minute
  *   <li> Units: Rotations per Minute
     */
    public void shooterOn(double velocity){
      m_velocity = velocity;
      _shooter.setControl(velocityController.withVelocity(-m_velocity/60).withFeedForward(0.5)); // Divide by 60 to go from RPM -> RPS 
      _shooter2.setControl(velocityController.withVelocity(m_velocity/60).withFeedForward(0.5)); // Divide by 60 to go from RPM -> RPS 
        // _shooter.set(0.8);
    }
    public void shooterOff(){
      _shooter.set(0);
      _shooter2.set(0); // Divide by 60 to go from RPM -> RPS 
    }

    /**Only turns on the top set of shooter wheels. Does not turn on bottom set. */
    public void shooterOnTop(double percentPower){
      // _shooter.setControl(velocityController.withVelocity(-m_velocity/60).withFeedForward(0.5)); // Divide by 60 to go from RPM -> RPS 
      _shooter.set(percentPower);
      _shooter2.set(0); // Divide by 60 to go from RPM -> RPS 
    }

    // public void shooterOnBottom(double percentPower){
    //   // _shooter.setControl(velocityController.withVelocity(-m_velocity/60).withFeedForward(0.5)); // Divide by 60 to go from RPM -> RPS 
    //   _shooter.set(0);
    //   _shooter2.set(percentPower); // Divide by 60 to go from RPM -> RPS 
    // }


    /**Velocity of the device in mechanism rotations per second. 
     * This can be the velocity of a remote sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs 
     * @return Velocity of Shooter Motor with ID 34
     * <li> Units: Rotations per Second
     */
    public double getVelocity1(){
        return _shooter.getVelocity().getValueAsDouble()*(1);
    }
  
    public double getVelocity2(){
        return _shooter2.getVelocity().getValueAsDouble()*(1);
      }
  
    public double getTargetVelocity(){
        return m_velocity;
    }

    public double getCurrent(){
        return _shooter.getStatorCurrent().getValue();
    }
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter1 RPM", getVelocity1()*60);
      SmartDashboard.putNumber("Shooter1 RPS", getVelocity1());
      SmartDashboard.putNumber("Shooter2 RPM", getVelocity2()*60);
      SmartDashboard.putNumber("Shooter2 RPS", getVelocity2());

      SmartDashboard.putNumber("Shooter TargetSpeed", m_velocity);
      
      Global_Variables.shooterTarget = m_velocity;
      Global_Variables.shooterSpeed = getVelocity2()*60;

    }
} 