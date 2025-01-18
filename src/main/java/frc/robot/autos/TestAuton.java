package frc.robot.autos;

import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuton extends SequentialCommandGroup {

    public TestAuton(Swerve swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        
        addCommands(
            new InstantCommand(() -> swerve.setGyro(-60)),
            new PathPlannerAuto("Red_Top_Bumper_0_5"),
            new InstantCommand(() -> swerve.setHeading(swerve.getGyroYaw()))
            // new RunCommand(() -> swerve.setDrive(-60,-60)).until(new Auton_Wait(10).getAsBoolean())
        );
    }
    
}
    