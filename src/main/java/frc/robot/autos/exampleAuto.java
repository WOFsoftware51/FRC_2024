package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(){
    //     TrajectoryConfig config =
    //         new TrajectoryConfig(
    //                 Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //             .setKinematics(Constants.Swerve.swerveKinematics);

    //     // An example trajectory to follow.  All units in meters.
    //     Trajectory exampleTrajectory =
    //         TrajectoryGenerator.generateTrajectory(
    //             // Start at the origin facing the +X direction
    //             new Pose2d(0, 0, new Rotation2d(0)),
    //             // Pass through these two interior waypoints, making an 's' curve path
    //             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //             // End 3 meters straight ahead of where we started, facing forward
    //             new Pose2d(3, 0, new Rotation2d(0)),
    //             config);

    //     var thetaController =
    //         new ProfiledPIDController(
    //             Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     SwerveControllerCommand swerveControllerCommand =
    //         new SwerveControllerCommand(
    //             exampleTrajectory,
    //             s_Swerve::getPose,
    //             Constants.Swerve.swerveKinematics,
    //             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //             thetaController,
    //             s_Swerve::setModuleStates,
    //             s_Swerve);


        addCommands(
        );
    // }
    }
}