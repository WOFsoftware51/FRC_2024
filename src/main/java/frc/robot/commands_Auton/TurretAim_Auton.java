package frc.robot.commands_Auton;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Turret;


import edu.wpi.first.wpilibj2.command.Command;


public class TurretAim_Auton extends Command {    
    public Turret m_turret;   
    public boolean endCommand = false;
    public int count = 0;

    
    public TurretAim_Auton(Turret turret) {
        this.m_turret = turret;
        addRequirements(m_turret);
        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
        m_turret.turret_init();
        endCommand = false;
         count = 0;
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/
        if(Global_Variables.tv == 1){
           m_turret.turret_Goto_angle(m_turret.getTurretAimTarget()+Global_Variables.tOffset_chooser.getSelected());//-2.75);
        
            if(Global_Variables.turretPos < Global_Variables.turretTarget + 1 && Global_Variables.turretPos > Global_Variables.turretTarget - 1 && count > 10){
                endCommand = true;
            }
            count++;
        }
        else{

        }



    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turretOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return endCommand;
    }
}