// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Arm;


public class Arm_Setposition extends Command
{

  private final Arm m_arm;
  
  private double armEncoder = 0.0;
  private double armCANCoder = 0.0;
  private double armSpeed = 0.0;

  private int button = 0;

  /** Creates a new Arm. */
  public Arm_Setposition(Arm arm, int Button) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    addRequirements(arm);
    this.button = Button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    armEncoder = m_arm.getArm_encoder();
    armCANCoder = m_arm.getArm_CANCoder();
    armSpeed = m_arm.Arm_Speed();

    m_arm.Arm_Goto_Angle(-m_arm.turretAngleToScore+90);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}