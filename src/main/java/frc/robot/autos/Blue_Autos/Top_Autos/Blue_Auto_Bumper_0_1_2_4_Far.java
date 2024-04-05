package frc.robot.autos.Blue_Autos.Top_Autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands.Transfer_IntakeShoot;
import frc.robot.commands_Auton.AutonSwerveAim;
import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.commands_Auton.TurretAim_Auton;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Blue_Auto_Bumper_0_1_2_4_Far extends SequentialCommandGroup {

    public Blue_Auto_Bumper_0_1_2_4_Far(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        
        addCommands(
            new InstantCommand(() -> swerve.setGyro(60)),
            new ParallelCommandGroup(
                aSub.auton_Shooter_Start(shooter),
                aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                    new Transfer_IntakeCommand(transfer),
                    new IntakeCommand(intake),
                    new PathPlannerAuto("Blue_Top_Bumper_0_1")
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret),
                new AutonSwerveAim(swerve, ()-> 0.0, ()-> 0.0)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_Top_Bumper_1_2")
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret),
                new AutonSwerveAim(swerve, ()-> 0.0, ()-> 0.0)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_Top_Bumper_2_4")
            ), //
            new ParallelCommandGroup(
                swerve.followTrajectoryCommand("Blue_Top_4_Shoot")
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret),
                new AutonSwerveAim(swerve, ()-> 0.0, ()-> 0.0)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new InstantCommand(() -> swerve.setHeading(swerve.getGyroYaw())),
            aSub.auton_Stop_Shooter(shooter)
        );    
    }
}