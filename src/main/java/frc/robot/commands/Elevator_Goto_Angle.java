package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Elevator_Goto_Angle extends Command {    
    public Elevator m_Elevator;   
    public double angle;
    private int m_button;
    private double elevatorTarget;
    private double elevatorPow = 0;

  

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
        if(Global_Variables.isIntaking){
            elevatorTarget = Constants.ELEVATOR_DEFAULT;
        }
        else{
            switch(m_button) {
                case Constants.Y_Button:   
                    elevatorTarget = Constants.ELEVATOR_AMP;
                    break;

                case Constants.B_Button:  
                elevatorTarget = Constants.ELEVATOR_TRAP;
                break;
                
                case Constants.X_Button:    
                elevatorTarget = Constants.ELEVATOR_FLOOR;
                    break;

                case Constants.A_Button:       
                    elevatorTarget = Constants.ELEVATOR_DEFAULT;
                    break;

                default:   
                    elevatorTarget = Constants.ELEVATOR_DEFAULT;
            }
        }



        if(elevatorTarget < m_Elevator.elevator_encoder()){

            if(elevatorTarget+5 > m_Elevator.elevator_encoder()){
                elevatorPow = 0.2;
            }
            else
            {
                elevatorPow = 1.0;
            }
       }
       else if(elevatorTarget > m_Elevator.elevator_encoder()){

            if(elevatorTarget-5 < m_Elevator.elevator_encoder()){
                elevatorPow = -0.2;
            }
            else
            {
                elevatorPow = -1.0;
            }
       }

       
        // if(elevatorTarget+1 < m_Elevator.elevator_encoder() && elevatorTarget-1 > m_Elevator.elevator_encoder()){
        //     elevatorPow = 0.0;
        // }
        

        m_Elevator.elevatorOn(elevatorPow);
        // if(m_Elevator.limitSwitchVal() && m_Elevator.getClosedLoopOutput() < 0){
        //     m_Elevator.elevatorOff();
        // }
        // else{
        //     m_Elevator.elevator_Goto_Position(elevatorTarget);
        // }
        SmartDashboard.putNumber("Elevator Target", elevatorTarget);
        SmartDashboard.putNumber("Elevator Pow", elevatorPow);

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