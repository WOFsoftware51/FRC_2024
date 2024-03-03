// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Transfer_IntakeShoot;
import frc.robot.commands.TurretAim;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.commands_Auton.Auton_Wait;

public class Auton_Subsystem extends SubsystemBase {
  /** Creates a new Auton_Subsystem. */
  public Auton_Subsystem() {}

  public boolean isShooting = false;


  public Command auton_Score(Turret turret, Transfer_Intake transfer, Shooter shooter){
    return new SequentialCommandGroup(
      auton_Aim(turret),
      new Auton_Wait(2),
      auton_TransferShoot(transfer, shooter)
);
  }

  public Command auton_Shooter(Shooter m_Shooter){
      return new ShootCommand(m_Shooter,()-> Constants.ShooterSpeeds.SHOOTER_AUTON_SPEED1).until(new Auton_Wait(100).getAsBoolean());    
  }

  public Command auton_Aim(Turret m_Turret){
    return new ParallelRaceGroup(
      new TurretAim(m_Turret).until(new Auton_Wait(100).getAsBoolean()),
      new WaitUntilCommand((()-> aimReady(m_Turret)))
    );
  }

  public boolean shotReady( Shooter mShooter){
    boolean bool = false;
    if(Math.abs(mShooter.getVelocity()) < Math.abs(mShooter.getTargetVelocity())+100 && Math.abs(mShooter.getVelocity()) > Math.abs(mShooter.getTargetVelocity())-100 ){
      bool = true;
    }
    else{
      bool = false;
    }

    return bool;
  }

  public boolean aimReady(Turret mTurret){
    boolean bool = false;
    if(Math.abs(mTurret.getTurretEncoder()) > Math.abs(mTurret.turretAngleToScore)-1 && mTurret.getTurretEncoder() < Math.abs(mTurret.turretAngleToScore)+1){
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
      return new IntakeCommand(intake);
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

  public Command ledRed(CANdle_Subsystem m_candle){
            return new SequentialCommandGroup( 
              new InstantCommand(
              ()-> m_candle.CANdle_init()
            ),
            new RunCommand(
              ()-> m_candle.CANdle_Red()
            )
            );
  }


  public Command ledBlue(CANdle_Subsystem m_candle){
            return new SequentialCommandGroup( 
              new InstantCommand(
              ()-> m_candle.CANdle_init()
            ), 
            new RunCommand(
              ()-> m_candle.CANdle_Blue()
            ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
