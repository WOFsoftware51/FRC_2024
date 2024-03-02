package frc.robot.autos.Red_Autos.Top_Autos;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Red_Auto_0_1_2_3 extends SequentialCommandGroup {

    public Red_Auto_0_1_2_3(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        addRequirements(swerve, turret, shooter, aSub, transfer);

        
        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelRaceGroup(
                aSub.auton_Shooter(shooter),
                aSub.auton_Intake(intake),
                new IntakeCommand(intake),
                new SequentialCommandGroup(
                    aSub.auton_Score(turret, transfer, shooter),
                    new PathPlannerAuto("Red_0_1"),
                    aSub.auton_Score(turret, transfer, shooter),
                    new PathPlannerAuto("Red_1_2"),
                    aSub.auton_Score(turret, transfer, shooter),
                    new PathPlannerAuto("Red_2_3"),
                    aSub.auton_Score(turret, transfer, shooter)
                )
            )
        );    
    }
}
