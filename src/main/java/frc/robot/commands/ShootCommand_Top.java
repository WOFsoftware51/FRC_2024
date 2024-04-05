package frc.robot.commands;

import frc.robot.Global_Variables;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class ShootCommand_Top extends Command {    
    private Shooter m_shooter;    
    private DoubleSupplier vSupplier;
    private double shotSpeed;

    // private Timer timer;


    /**@param percentSupplier The percent power of the Top Shooter
     * @param shooter The required subsystem
     */
    public ShootCommand_Top(Shooter shooter, DoubleSupplier percentSupplier) {
        this.m_shooter = shooter;
        this.vSupplier = percentSupplier;
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
        shotSpeed = vSupplier.getAsDouble();
        
        m_shooter.shooterOnTop(shotSpeed);
        Global_Variables.isShooting = true;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shooterOff();
        Global_Variables.isShooting = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}