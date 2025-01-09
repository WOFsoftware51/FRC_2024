// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Music extends Command
{
  private final Swerve s_swerve;
  
  public Music(Swerve swerve)//, Arm arm, Wrist wrist, Intake intake, Extend extend, boolean on)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_swerve = swerve;
    addRequirements(s_swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    s_swerve.music_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      s_swerve.play_music();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  @Override
  public boolean runsWhenDisabled() 
  {
    return true;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() 
  {
    return InterruptionBehavior.kCancelSelf;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
      return false;
  }
}
