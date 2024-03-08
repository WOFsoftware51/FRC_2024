package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class ShootCommand_Top extends Command {    
    private Shooter m_shooter;    
    private DoubleSupplier vSupplier;
    private DoubleSupplier vSupplier2;

    // private Timer timer;

    public ShootCommand_Top(Shooter shooter) {
        this.m_shooter = shooter;
        addRequirements(m_shooter);

        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
       m_shooter.shooter_init();
    }


    @Override
    public void execute() {        
        m_shooter.shooterOnTop();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shooterOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}