package frc.robot.autos;

import frc.robot.Global_Variables;
import frc.robot.commands.AutonSwerve;
import frc.robot.commands.Auton_Wait;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TimeBasedAutoTest extends SequentialCommandGroup {
    private final Swerve s_Swerve;

    public TimeBasedAutoTest(Swerve swerve){
        this.s_Swerve = swerve;


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new ParallelRaceGroup(
                new AutonSwerve(s_Swerve, ()-> 0.5, ()-> 0, ()-> 0, ()-> false),
                new WaitUntilCommand(()-> !new Auton_Wait(1).isFinished())
            ),
            new AutonSwerve(s_Swerve, ()-> 0.0, ()-> 0, ()-> 0, ()-> false)

            );
    }
}   