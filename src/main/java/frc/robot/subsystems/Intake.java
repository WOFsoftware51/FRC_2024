package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake extends SubsystemBase {


    private TalonFX _intake = new TalonFX(Constants.intake, Constants.CANIVORE_NAME);
  
    public Intake() 
    {
      _intake.setNeutralMode(NeutralModeValue.Coast);
      _intake.setInverted(true);
    }

    public void intake_init() 
    {
      // _intake.setNeutralMode(NeutralModeValue.Coast);
      // _intake.setInverted(true);
    }
    
    public void intakeOn(){
       _intake.set(-1.0);//-1.0);

    }

    public void intakeReverse(){
      _intake.set(1.0);
    }
  
    public void intakeOff(){
      _intake.set(0);
    }
    
    
    @Override
    public void periodic(){
      
    }
  
  }