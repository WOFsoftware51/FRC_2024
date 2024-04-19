// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands.Transfer_IntakeShoot;
import frc.robot.commands.TurretAim;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.commands_Auton.ShootCommand_Start;
import frc.robot.commands_Auton.Transfer_IntakeShoot_Auton;

public class Auton_Subsystem extends SubsystemBase {
  /** Creates a new Auton_Subsystem. */
  public Auton_Subsystem(){
    
  }

  private boolean isShooting = false;


  public Command auton_Score(Turret turret, Transfer_Intake transfer, Shooter shooter, double turretAngle){
    return new SequentialCommandGroup(
      auton_Aim(turret, turretAngle),
      new Auton_Wait(2),
      auton_TransferShoot(transfer, shooter)
);
  }

  public Command auton_Limelight_Score(Turret turret, Transfer_Intake transfer, Shooter shooter){
    return new SequentialCommandGroup(
      auton_Limelight_Aim(turret),
      new Auton_Wait(2),
      auton_TransferShoot(transfer, shooter)
);
  }


  public Command auton_Shooter(Shooter m_Shooter){
      return new ShootCommand(m_Shooter,()-> Constants.ShooterSpeeds.SHOOTER_AUTON_SPEED1);    
  }

  public Command auton_Aim(Turret m_Turret, double turretAngle){
    return new ParallelRaceGroup(
      new Turret_Goto_Angle(m_Turret, turretAngle).until(new Auton_Wait(100).getAsBoolean()),
      new WaitUntilCommand((()-> turretAimReady(m_Turret)))
    );
  }

  public Command auton_Limelight_Aim(Turret m_Turret){
    return new ParallelRaceGroup(
      new TurretAim(m_Turret).until(new Auton_Wait(100).getAsBoolean()),
      new WaitUntilCommand((()-> turretAimReady(m_Turret)))
    );
  }

  public Command auton_Shooter_Start(Shooter m_Shooter){
    return new ShootCommand_Start(m_Shooter,()-> Constants.ShooterSpeeds.SHOOTER_AUTON_SPEED1).until(()-> shotReady(m_Shooter));
  }
/**Command is interrupted when turret is within 1 degree of the angleTarget */
  public Command auton_Turret_Start(Turret m_Turret, double angleTarget){
    return new Turret_Goto_Angle(m_Turret, angleTarget).until(()-> turretAimReady(m_Turret));
  }

  /**Shoot until sensor detects  */
  public Command auton_Shoot(Transfer_Intake transfer){
    return new Transfer_IntakeShoot_Auton(transfer).until(intakeReady());
  }

  public BooleanSupplier intakeReady(){
    return ()-> {
      if(Global_Variables.getSensorVal() == 1){
        return false;
      }
      else{
        return true;
      }

    };
  }

  public boolean shotReady( Shooter mShooter){
    boolean bool = false;

    //TODO getVelocity1() is in RPS. Change to RPM

    if(Global_Variables.shooterSpeed > 3700){
      bool = true;
    }
    else{
      bool = false;
    }


    // if(mShooter.getVelocity1() > 3500){
    //   bool = true;
    // }
    // else{
    //   bool = false;
    // }

    return bool;
  }


  public boolean turretAimReady(Turret mTurret){
    boolean bool = false;
    if(Global_Variables.turretPos < Global_Variables.turretTarget + 1 && Global_Variables.turretPos > Global_Variables.turretTarget - 1){
      bool = true;
    }
    else{
      bool = false;
    }

    return bool;
  }

  public Command auton_TransferShoot(Transfer_Intake mTransfer, Shooter shooter){
    return new  ParallelRaceGroup(
      new Transfer_IntakeShoot(mTransfer),
      new WaitUntilCommand(()-> isShooting(shooter))
    );
  }

  public Command auton_Intake(Intake intake){
      return new RunCommand(()-> intake.intakeOn()).until(new Auton_Wait(100).getAsBoolean());
  }


  public Command auton_Stop_Shooter(Shooter shooter){
    return new RunCommand(()-> shooter.shooterOff()).until(new Auton_Wait(5).getAsBoolean());
  }

  public Command auton_Stop_Transfer_Shooter(Transfer_Intake transfer){
    return new RunCommand(()-> transfer.shooter_transferOff()).until(new Auton_Wait(5).getAsBoolean());
  }


  public boolean isShooting(Shooter shooter){
    if(shooter.getCurrent() > Constants.AutoConstants.shooterCurrentSpike){
      isShooting = true;
    }
    else{ 
      isShooting = false;
    }

    return isShooting;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
