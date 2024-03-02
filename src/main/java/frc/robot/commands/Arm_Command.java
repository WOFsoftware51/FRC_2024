// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Arm_Command extends Command
{

  private final DoubleSupplier m_translationXSupplier;
  private final Arm m_arm;
  private double armEncoder = 0.0;
  // private double armCANCoder = 0.0;
  private double armSpeed = 0.0;
  private int count = 0;

  /** Creates a new Arm. */
  public Arm_Command(Arm arm, DoubleSupplier translationXSupplier) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
     this.m_arm = arm;
    addRequirements(arm);
    this.m_translationXSupplier = translationXSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
   // m_arm.arm_resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    armEncoder = m_arm.getArm_encoder();
    // armCANCoder = m_arm.getArm_CANCoder()%360;
    armSpeed = m_arm.Arm_Speed();
   SmartDashboard.putNumber("Count", count);

    if(count > 2 && (armEncoder < -20 || armEncoder > 20))
    {
      m_arm.updateEncoder();
      count = 0;
    }
    else
    {
      count++;
    }
  
     if(armEncoder > 115)
    {
      if(m_translationXSupplier.getAsDouble() < 0)
      {
        m_arm.arm_on(m_translationXSupplier.getAsDouble()*0.5);
      }
      else
      {
        m_arm.arm_off();
      }
    }
    else if(armEncoder < -109)
    {
      if(m_translationXSupplier.getAsDouble() > 0)
      {
        m_arm.arm_on(m_translationXSupplier.getAsDouble()*0.5);
      }
      else
      {
        m_arm.arm_off();
      }
    }
    else
    {
      m_arm.arm_on(m_translationXSupplier.getAsDouble()*0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
