package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake extends SubsystemBase {


    private TalonFX _intake = new TalonFX(Constants.intake);
  

    public void intake_init() 
    {
      _intake.setNeutralMode(NeutralModeValue.Brake);
      _intake.setInverted(true);

      // transfer_intake.setNeutralMode(NeutralModeValue.Brake);
      // transfer_intake.setInverted(false);

    }
    
    public void intakeOn(){
       _intake.set(1.0);
    }

    public void intakeReverse(){
      _intake.set(-1.0);
    }
  
    public void intakeOff(){
        _intake.set(0);
    }
    
    
    @Override
    public void periodic(){
      
    }
  
  }