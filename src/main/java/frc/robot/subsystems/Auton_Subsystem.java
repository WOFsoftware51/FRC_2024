// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Global_Variables;

public class Auton_Subsystem extends SubsystemBase {
  /** Creates a new Auton_Subsystem. */
  public Auton_Subsystem() {}


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
