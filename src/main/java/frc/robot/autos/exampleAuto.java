package frc.robot.autos;

import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class exampleAuto extends SequentialCommandGroup {
    Swerve s_Swerve;
    public exampleAuto(Swerve swerve){
        this.s_Swerve = swerve;
        addRequirements(s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new Auton_Wait(100),
            new PathPlannerAuto("Test_Auto")
        );
    }
}