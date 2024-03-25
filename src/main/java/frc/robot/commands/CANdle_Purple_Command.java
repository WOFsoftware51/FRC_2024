// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdle_Subsystem;

public class CANdle_Purple_Command extends Command {
  /** Creates a new CANdle_Command. */
  CANdle_Subsystem m_candle;
  public CANdle_Purple_Command(CANdle_Subsystem candle) 
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
    m_candle.CANdle_Purple();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
     m_candle.CANdle_Default();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
