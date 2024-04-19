package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Turret;


import edu.wpi.first.wpilibj2.command.Command;


public class TurretAim extends Command {    
    public Turret m_turret;   

    public TurretAim(Turret turret) {
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
        if(Global_Variables.tv == 1){
           m_turret.turret_Goto_angle(m_turret.getTurretAimTarget()+Global_Variables.tOffset_chooser.getSelected());//-2.75
        }
        else if(Global_Variables.tv == 0){
           m_turret.turret_Goto_angle(Constants.Turret.TURRET_DEFAULT_POSITION);
        }
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