package frc.robot.autos.Red_Autos.Middle_Autos;

import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeShoot;
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

public class Red_Auto_Middle_0_2 extends SequentialCommandGroup {

    public Red_Auto_Middle_0_2(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);
        
        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shooter_Start(shooter)// aSub.auton_Shooter(shooter)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(125),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeShoot(transfer),
                new PathPlannerAuto("Red_0_2"),
                new IntakeCommand(intake)
            ),
            // new ParallelRaceGroup(
            //     new Auton_Wait(25), // From 50 -> 25 
            //     new IntakeCommand(intake)
            // ),
            new PathPlannerAuto("Red_2_0"),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            aSub.auton_Stop_Shooter(shooter)        
        );    
    }
}


