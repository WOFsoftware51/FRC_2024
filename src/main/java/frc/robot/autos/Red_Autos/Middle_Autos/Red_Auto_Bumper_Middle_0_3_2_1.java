package frc.robot.autos.Red_Autos.Middle_Autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands_Auton.Auton_Wait;
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

public class Red_Auto_Bumper_Middle_0_3_2_1 extends SequentialCommandGroup {

    public Red_Auto_Bumper_Middle_0_3_2_1(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelCommandGroup(
                aSub.auton_Shooter_Start(shooter),
                aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(125),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                new PathPlannerAuto("Red_Middle_Bumper_0_3")
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Red_Middle_Bumper_3_0")
            ),
            aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Red_0_2")
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Red_2_0")
            ),
            aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Red_Middle_Bumper_0_1")
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Red_Middle_Bumper_1_0")
            ),
            aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new InstantCommand(() -> swerve.setHeading(swerve.getGyroYaw())),
            aSub.auton_Stop_Shooter(shooter)
        );
    }
}