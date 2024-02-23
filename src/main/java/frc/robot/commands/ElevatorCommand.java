// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command
{
  private final Elevator m_elevator;
  private Boolean in;
  public ElevatorCommand(Elevator elevator, Boolean _in) 
  { 
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator = elevator;
    addRequirements(elevator);
    this.in = _in;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      m_elevator.elevator_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {    
    if(in)
    {
      m_elevator.elevatorOn();
    }
    else if(in==false)
    {
      m_elevator.elevatorReverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_elevator.elevatorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
