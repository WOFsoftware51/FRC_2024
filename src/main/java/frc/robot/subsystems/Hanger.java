package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Hanger extends SubsystemBase {


    private TalonFX _hanger = new TalonFX(Constants.hanger,  Constants.CANIVORE_NAME);
    private TalonFX _hanger2 = new TalonFX(Constants.hanger2, Constants.CANIVORE_NAME);

    public Hanger() {
      _hanger.setNeutralMode(NeutralModeValue.Brake);
      _hanger2.setNeutralMode(NeutralModeValue.Brake);
      _hanger.setInverted(true);
      _hanger2.setInverted(false);
    }

    public void hanger_init() {
      // _hanger.setNeutralMode(NeutralModeValue.Brake);
      // _hanger2.setNeutralMode(NeutralModeValue.Brake);
      // _hanger.setInverted(true);
      // _hanger2.setInverted(false);
    }
    
    public void hangerOn(){
      _hanger.set(0.6);
      _hanger2.set(0.6);
    }

    public void hanger1On(double x){
      _hanger.set(x);
    }

    public void hanger2On(double x){
      _hanger2.set(x);
    }


    public void hangerJoystick(double x){
      _hanger.set(x);
      _hanger2.set(x);
    }

    public void hangerSeperateControl(double x1, double x2){
      _hanger.set(x1*0.5);
      _hanger2.set(x2*0.5);
    }

    public void hangerReverse(){
      _hanger.set(-0.6);
      _hanger2.set(-0.6);
    }
  
    public void hangerOff(){
      _hanger.set(0);
      _hanger2.set(0);
    }

    public void resetEncoder1()
    {
      _hanger.setPosition(0);
    }

    public void resetEncoder2()
    {
      _hanger2.setPosition(0);
    }

    public double getEncoder1(){
      return _hanger.getPosition().getValue();
    }

    public double getEncoder2(){
      return _hanger2.getPosition().getValue();
    }

    
    @Override
    public void periodic(){
      // if(_hanger.getForwardLimit().getValue()==ForwardLimitValue.Open)
      // {
      //   resetEncoder1();
      // }
      // if(_hanger2.getForwardLimit().getValue()==ForwardLimitValue.Open)
      // {
      //   resetEncoder2();
      // }
      
      SmartDashboard.putNumber("Hangar 1 encoder", getEncoder1());
      SmartDashboard.putNumber("Hangar 2 encoder", getEncoder2());
  
    }
  
  }