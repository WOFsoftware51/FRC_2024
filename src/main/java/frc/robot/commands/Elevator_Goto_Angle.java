package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


import edu.wpi.first.wpilibj2.command.Command;


public class Elevator_Goto_Angle extends Command {    
    public Elevator m_Elevator;   
    public double angle;
    private int m_button;
    private double elevatorTarget;

  

    public Elevator_Goto_Angle(Elevator Elevator, int button) {
        this.m_Elevator = Elevator;
        addRequirements(m_Elevator);
        this.m_button = button;
    }

    @Override
    public void initialize() {
        m_Elevator.elevator_init();
    }

    @Override
    public void execute() {
        switch(m_button) {
            case Constants.A_Button:   ////////////////////////////////////////////////////////////
                elevatorTarget = Constants.ELEVATOR_AMP;
                break;

            case Constants.B_Button:  ////////////////////////////////////////////////////////////
                elevatorTarget = Constants.ELEVATOR_SPEAKER;
                break;

            case Constants.X_Button:    ////////////////////////////////////////////////////////////
                elevatorTarget = Constants.ELEVATOR_TRAP;
                break;

            case Constants.Y_Button:       ////////////////////////////////////////////////////////////
                elevatorTarget = Constants.ELEVATOR_DEFAULT;
                break;

            default:   ////////////////////////////////////////////////////////////
                elevatorTarget = Constants.ELEVATOR_DEFAULT;
        }
      
        m_Elevator.elevator_Goto_Position(elevatorTarget);
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.elevatorOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}