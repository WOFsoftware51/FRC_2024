// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command
{
  private final Elevator m_elevator;
  private Boolean in;
  private DoubleSupplier joystick;
  private double joystickFixed= 0;


  public ElevatorCommand(Elevator elevator, DoubleSupplier jDoubleSupplier) 
  { 
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator = elevator;
    addRequirements(elevator);
    this.joystick = jDoubleSupplier;
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
    joystickFixed = MathUtil.applyDeadband(joystick.getAsDouble(), Constants.stickDeadband);


      if(m_elevator.limitSwitchVal() == -1 && joystickFixed < 0){
        m_elevator.elevatorOff();
      }
      else if(m_elevator.elevator_encoder() <= Constants.Elevator_Highest_Point && joystickFixed > 0){
        m_elevator.elevatorOff();
      }
      else{
        m_elevator.elevatorOn(joystickFixed);
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
