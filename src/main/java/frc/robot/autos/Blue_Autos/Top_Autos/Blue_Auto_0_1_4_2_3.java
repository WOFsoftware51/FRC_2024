package frc.robot.autos.Blue_Autos.Top_Autos;

import frc.robot.Constants;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Blue_Auto_0_1_4_2_3 extends SequentialCommandGroup {


    public Blue_Auto_0_1_4_2_3(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        
        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new PathPlannerAuto("Blue_Top_0_Rotate"),
            new ParallelRaceGroup(
                aSub.auton_Shooter(shooter),
                aSub.auton_Intake(intake),
                new SequentialCommandGroup(
                    aSub.auton_Score(turret, transfer, shooter, Constants.AutonTurretPositions.Top.Position_Start),
                    new PathPlannerAuto("Blue_0_1"),
                    aSub.auton_Score(turret, transfer, shooter, Constants.AutonTurretPositions.Top.Position_1),
                    new PathPlannerAuto("Blue_1_4"),
                    new PathPlannerAuto("Blue_4_1"),
                    aSub.auton_Score(turret, transfer, shooter, Constants.AutonTurretPositions.Top.Position_1),
                    new PathPlannerAuto("Blue_1_2"),
                    aSub.auton_Score(turret, transfer, shooter, Constants.AutonTurretPositions.Top.Position_2),
                    new PathPlannerAuto("Blue_2_3"),
                    aSub.auton_Score(turret, transfer, shooter, Constants.AutonTurretPositions.Top.Position_2)
                )
            )
        );
    }
}
