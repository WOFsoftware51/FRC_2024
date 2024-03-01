package frc.robot.autos;

import frc.robot.commands.Auton_Wait;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.CANdle_Subsystem;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Red_Middle_Auto extends SequentialCommandGroup {
    Swerve s_Swerve;
    public Red_Middle_Auto(Swerve swerve, Auton_Subsystem aSub, CANdle_Subsystem m_Candle){
        this.s_Swerve = swerve;
        addRequirements(s_Swerve, aSub, m_Candle);

        
        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            aSub.ledRed(m_Candle)            
        );
    }
}