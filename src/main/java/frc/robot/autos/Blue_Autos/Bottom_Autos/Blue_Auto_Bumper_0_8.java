package frc.robot.autos.Blue_Autos.Bottom_Autos;

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

public class Blue_Auto_Bumper_0_8 extends SequentialCommandGroup {

    public Blue_Auto_Bumper_0_8(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        
        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shooter(shooter)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(75),
                aSub.auton_Shooter(shooter),
                new Transfer_IntakeCommand(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                new PathPlannerAuto("Blue_Bottom_Bumper_0_8")
            ),
            new ParallelRaceGroup(
                new Auton_Wait(25),
                new IntakeCommand(intake)
            ),
            new PathPlannerAuto("Blue_Bottom_Bumper_8_0"),
            new ParallelRaceGroup(
                new Auton_Wait(75),
                aSub.auton_Shooter(shooter)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(75),
                aSub.auton_Shooter(shooter),
                new Transfer_IntakeCommand(transfer)
            ),
            // new InstantCommand(() -> swerve.setYawWrapped(-59.60)),
            aSub.auton_Stop_Shooter(shooter)

        );
    }
}