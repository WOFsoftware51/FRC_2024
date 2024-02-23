package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake extends SubsystemBase {


    private TalonFX _intake = new TalonFX(Constants.intake);
    private TalonFX transfer_intake = new TalonFX(Constants.transfer_intake);
  

    public void intake_init() 
    {
      _intake.setNeutralMode(NeutralModeValue.Brake);
      _intake.setInverted(true);

      transfer_intake.setNeutralMode(NeutralModeValue.Brake);
      transfer_intake.setInverted(false);

    }
    
    public void intakeOn(){
        _intake.set(0.6);
    }

    public void intakeReverse(){
      _intake.set(-0.6);
    }
  
    public void intakeOff(){
        _intake.set(0);
    }


    public void transferOn(){
        transfer_intake.set(0.6);
    }

    public void transferReverse(){
      transfer_intake.set(-0.6);
    }
  
    public void transferOff(){
        transfer_intake.set(0);
    }
    
    
    
    @Override
    public void periodic(){
      
    }
  
  }