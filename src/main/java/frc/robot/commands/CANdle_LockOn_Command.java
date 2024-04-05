// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Global_Variables;
import frc.robot.subsystems.CANdle_Subsystem;

public class CANdle_LockOn_Command extends Command {
  /** Creates a new CANdle_Command. */
  CANdle_Subsystem m_candle;
  private boolean turretLockOn = false;
  private boolean swerveLockOn = false;
  private boolean shooterLockOn = false;


  public CANdle_LockOn_Command(CANdle_Subsystem candle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_candle = candle;
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_candle.CANdle_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if(Global_Variables.turretPos < Global_Variables.turretTarget + 1 && Global_Variables.turretPos > Global_Variables.turretTarget - 1){
      turretLockOn = true;
    }
    else{
      turretLockOn = false;
    }

    if(Global_Variables.tv==1){
      if(Global_Variables.tx < 4.0 && Global_Variables.tx > -4.0){
        swerveLockOn = true;
      }
      else{
        swerveLockOn = false;
      }
    }
    else{
      // if((Global_Variables.yawFixed > 357 || Global_Variables.yawFixed < 3)){
      //   swerveLockOn = true;
      // }
      // else{
        swerveLockOn = false;
      // }
    }

    if(Global_Variables.shooterSpeed > Global_Variables.shooterTarget-500 && Global_Variables.shooterSpeed < Global_Variables.shooterTarget + 500){
      shooterLockOn = true;
    }
    else{
      shooterLockOn = false;
    }

    if(turretLockOn && swerveLockOn && shooterLockOn){
      m_candle.CANdle_Solid_Green();
    }
    else{
      m_candle.CANdle_Red();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
     m_candle.CANdle_Default();
  }
  
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
