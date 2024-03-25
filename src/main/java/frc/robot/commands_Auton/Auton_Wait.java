// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands_Auton;

import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auton_Wait extends Command {
  /** Creates a new Auton_Wait. */

  private double time = 0.0;
  private double counter = 0.0;
  private Boolean end = false;
  /**50 units of time = 1 second */
  public Auton_Wait(double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    end = false;
    counter = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Auton Counter", counter);
    SmartDashboard.putNumber("Auton Time", time);


    if(counter<time)
    {
      counter++;
    }
    else
    {
      end = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public BooleanSupplier getAsBoolean(){
    return ()-> end;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
