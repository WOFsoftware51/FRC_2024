package frc.robot.autos;

import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Square_Auto extends SequentialCommandGroup {

    Swerve s_Swerve;
    public Square_Auto(Swerve swerve){
        this.s_Swerve = swerve;
        addRequirements(s_Swerve);

        String path1 = "Square_Path";

        Command followPath1 = s_Swerve.followTrajectoryCommand(path1, false);

        addCommands(
            followPath1
        );
    }
}