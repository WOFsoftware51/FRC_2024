package frc.robot.commands_Auton;

import frc.robot.subsystems.Turret;


import edu.wpi.first.wpilibj2.command.Command;


public class Turret_Goto_Angle_Auton extends Command {    
    public Turret m_turret;   
    public double angle;
    private boolean endCommand = false;
    private int time = 0;
    private int timeElapsed = 0;

    public Turret_Goto_Angle_Auton(Turret turret, double _angle, int duration) {
        this.m_turret = turret;
        addRequirements(m_turret);
        this.angle = _angle;
        this.time = duration;
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
       m_turret.turret_init();
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/

        if(timeElapsed<time){
            timeElapsed++;
        }
        else{
            endCommand = true;
        }
  
        m_turret.turret_Goto_angle(angle);
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