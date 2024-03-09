package frc.robot.commands;

import frc.robot.subsystems.Turret;


import edu.wpi.first.wpilibj2.command.Command;


public class Turret_Goto_Angle extends Command {    
    public Turret m_turret;   
    public double angle;

    public Turret_Goto_Angle(Turret turret, double _angle) {
        this.m_turret = turret;
        addRequirements(m_turret);
        this.angle = _angle;
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
    

       m_turret.turret_Goto_angle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turretOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}