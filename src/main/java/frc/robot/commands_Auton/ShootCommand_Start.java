package frc.robot.commands_Auton;

import frc.robot.Global_Variables;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class ShootCommand_Start extends Command {    
    private Shooter m_shooter;    
    private DoubleSupplier vSupplier;
    private double shotSpeed;
    // private Timer timer;

    /**Doesn't Stop Shooting When Interupted */
    public ShootCommand_Start(Shooter shooter, DoubleSupplier supplierV) {
        this.m_shooter = shooter;
        this.vSupplier = supplierV;
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
        
        m_shooter.shooterOn(shotSpeed);
        Global_Variables.isShooting = true;
    }

    @Override
    public void end(boolean interrupted) {
        Global_Variables.isShooting = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}