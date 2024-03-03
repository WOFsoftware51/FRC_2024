package frc.robot.autos.Blue_Autos.Middle_Autos;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Blue_Middle_Auto_0_1 extends SequentialCommandGroup {
    Swerve s_Swerve;
    Turret m_Turret;
    Shooter m_Shooter;

    public Blue_Middle_Auto_0_1(Swerve swerve, Turret turret, Shooter shooter){
        this.s_Swerve = swerve;
        this.m_Turret = turret;
        this.m_Shooter = shooter;
        addRequirements(s_Swerve, m_Turret, m_Shooter);

        
        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new ParallelRaceGroup(
                new Turret_Goto_Angle(m_Turret, Constants.AutoPositions.TURRET_AUTON_POSITION2).until(new Auton_Wait(100).getAsBoolean()),
                new ShootCommand(m_Shooter,()-> Constants.ShooterSpeeds.SHOOTER_AUTON_SPEED1).until(new Auton_Wait(100).getAsBoolean()),
                new WaitUntilCommand((()->  ((Global_Variables.getSensorVal() == 1) ? true : false)))
            ),
            new Auton_Wait(2),
            new Turret_Goto_Angle(m_Turret, Constants.AutoPositions.TURRET_AUTON_POSITION2).until(new Auton_Wait(100).getAsBoolean()),
            new PathPlannerAuto("Test_Auto")
        );
    }
}
