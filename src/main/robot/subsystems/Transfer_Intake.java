package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Global_Variables;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Transfer_Intake extends SubsystemBase {


    private TalonFX transfer_intake = new TalonFX(Constants.transfer_intake, Constants.CANIVORE_NAME);
    private TalonFX transfer_shooter = new TalonFX(Constants.transfer_shooter, Constants.CANIVORE_NAME);

    public void intake_init() 
    {
      transfer_intake.setNeutralMode(NeutralModeValue.Brake);
      transfer_intake.setInverted(false);
      transfer_shooter.setNeutralMode(NeutralModeValue.Brake);
      transfer_shooter.setInverted(false);

    }

    public void transferOn(){
        transfer_intake.set(-1);
    }

    public void transferReverse(){
      transfer_intake.set(1);
    }
  
    public void transferOff(){
        transfer_intake.set(0);
    }

    public void shooter_transferOn(){
        transfer_shooter.set(-1);
    }

    public void shooter_transferReverse(){
      transfer_shooter.set(0.8);
    }
  
    public void shooter_transferOff(){
        transfer_shooter.set(0);
    }
    
    
    
    @Override
    public void periodic(){
            SmartDashboard.putNumber("Prox sensor", Global_Variables.getSensorVal());
    }
  
  }