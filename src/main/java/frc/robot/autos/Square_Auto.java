package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Square_Auto extends SequentialCommandGroup {
    private final Swerve s_Swerve;

    public Square_Auto(Swerve swerve){
        this.s_Swerve = swerve;

        String path1 = "Square_Path";


        Command followPath1 = s_Swerve.followTrajectoryCommand(path1, false);

        addCommands(
            followPath1
        );
    }
}