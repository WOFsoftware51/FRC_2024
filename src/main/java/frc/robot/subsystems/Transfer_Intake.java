package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Transfer_Intake extends SubsystemBase {


    private TalonFX transfer_intake = new TalonFX(Constants.transfer_intake);
    private TalonFX transfer_shooter = new TalonFX(Constants.transfer_shooter);

    public void intake_init() 
    {
      transfer_intake.setNeutralMode(NeutralModeValue.Brake);
      transfer_intake.setInverted(false);
      transfer_shooter.setNeutralMode(NeutralModeValue.Brake);
      transfer_shooter.setInverted(false);

    }

    public void transferOn(){
        transfer_intake.set(0.8);
    }

    public void transferReverse(){
      transfer_intake.set(-0.8);
    }
  
    public void transferOff(){
        transfer_intake.set(0);
    }

    public void shooter_transferOn(){
        transfer_intake.set(0.8);
    }

    public void shooter_transferReverse(){
      transfer_intake.set(-0.8);
    }
  
    public void shooter_transferOff(){
        transfer_intake.set(0);
    }
    
    
    
    @Override
    public void periodic(){
      
    }
  
  }