package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Hanger;


public class HangerCommand extends Command {    
    public Hanger m_Hanger;   
    public boolean _up = true;
    public double hanger1Pow = 0.5;
    public double hanger2Pow = 0.5;
    public double hangerDirection = 1;
    // private Timer timer;

    public HangerCommand(Hanger hanger, boolean up) {
        this.m_Hanger = hanger;
        addRequirements(m_Hanger);
        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
       m_Hanger.hanger_init();
    }

    @Override
    public void execute() {

        if(_up){
            hangerDirection = 1;
        }
        else{
            hangerDirection = -1;
        }

        if(Global_Variables.roll < -7.5){
            hanger1Pow = 0.5;
            hanger2Pow = 0.0;
        }
        else if(Global_Variables.roll > 7.5){
            hanger1Pow = 0.0;
            hanger2Pow = 0.5;
        }
        else if(Global_Variables.roll >= -7.5 && Global_Variables.roll <= 7.5){
            hanger1Pow = 0.5;
            hanger2Pow = 0.5;
        }

        m_Hanger.hangerSeperateControl(hanger1Pow*hangerDirection, hanger2Pow*hangerDirection);
    }

    @Override
    public void end(boolean interrupted) {
        m_Hanger.hangerOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}