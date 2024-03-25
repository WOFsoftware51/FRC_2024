package frc.robot.autos;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands.Transfer_IntakeShoot;
import frc.robot.commands_Auton.Auton_Wait;
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

public class Shoot_Only_Auto extends SequentialCommandGroup {

    public Shoot_Only_Auto(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake); //TODO


        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelRaceGroup(
                new Auton_Wait(75),
                aSub.auton_Shooter_Start(shooter)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(125),
                aSub.auton_Shoot(transfer)
            ),
            aSub.auton_Stop_Shooter(shooter)
        );
    }
}
