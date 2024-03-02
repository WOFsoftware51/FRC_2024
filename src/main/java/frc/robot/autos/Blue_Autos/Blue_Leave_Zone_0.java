package frc.robot.autos.Blue_Autos;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Blue_Leave_Zone_0 extends SequentialCommandGroup {


    public Blue_Leave_Zone_0(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer){
        
        addRequirements(swerve, turret, shooter, aSub, transfer);

        
        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelRaceGroup(
                aSub.auton_Shooter(shooter),
            new SequentialCommandGroup(
                    new PathPlannerAuto("Blue_Leave_Zone_0"),
                    aSub.auton_Score(turret, transfer, shooter)
                )
            )
        );
    }
}
