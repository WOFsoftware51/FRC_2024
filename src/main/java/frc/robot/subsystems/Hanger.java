package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Hanger extends SubsystemBase {


    private TalonFX _hanger = new TalonFX(Constants.hanger,  Constants.CANIVORE_NAME);
    private TalonFX _hanger2 = new TalonFX(Constants.hanger2, Constants.CANIVORE_NAME);

  
    public void hanger_init() {
      _hanger.setNeutralMode(NeutralModeValue.Brake);
      _hanger2.setNeutralMode(NeutralModeValue.Brake);
      _hanger.setInverted(true);
      _hanger2.setInverted(false);
    }
    
    public void hangerOn(){
      _hanger.set(0.6);
      _hanger2.set(0.6);
    }

    public void hangerJoystick(double x){
      _hanger.set(x);
      _hanger2.set(x);
    }

    public void hangerSeperateControl(double x1, double x2){
      _hanger.set(x1);
      _hanger2.set(x2);
    }

    public void hangerReverse(){
      _hanger.set(-0.6);
      _hanger2.set(-0.6);
    }
  
    public void hangerOff(){
      _hanger.set(0);
      _hanger2.set(0);
    }

    public void hanger_resetEncoder1()
    {
      _hanger.setPosition(0);
    }

    public void hanger_resetEncoder2()
    {
      _hanger2.setPosition(0);
    }
    
    @Override
    public void periodic(){
    //   if(_hanger.getForwardLimit().getValue()==ForwardLimitValue.Open)
    //   {
    //     hanger_resetEncoder1();
    //   }
    //   if(_hanger2.getForwardLimit().getValue()==ForwardLimitValue.Open)
    //   {
    //     hanger_resetEncoder2();
    //   }
    }
  
  }