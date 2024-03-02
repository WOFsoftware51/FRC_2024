package frc.robot.commands_Auton;

import frc.robot.subsystems.Turret;


import edu.wpi.first.wpilibj2.command.Command;


public class TurretAim_Auton extends Command {    
    public Turret m_turret;   

    public TurretAim_Auton(Turret turret) {
        this.m_turret = turret;
        addRequirements(m_turret);
        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
       m_turret.turret_init();
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/
       m_turret.turret_Goto_angle(m_turret.turretAngleToScore);
    }

    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}