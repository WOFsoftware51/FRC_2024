// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Music extends Command {

  Swerve s_Swerve;
  /** Creates a new Music. */
  public Music(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = swerve;
    addRequirements(s_Swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    s_Swerve.music_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_Swerve.play_music();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    s_Swerve.stop_music();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
