package frc.robot.autos.Red_Autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class Red_Leave_Zone extends SequentialCommandGroup {
    Swerve s_Swerve;

    public Red_Leave_Zone(Swerve swerve){
        this.s_Swerve = swerve;
        addRequirements(s_Swerve);

        
        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new PathPlannerAuto("Red_Leave_Zone")
        );
    }
}
