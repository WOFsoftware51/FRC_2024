package frc.robot.commands;

import frc.robot.subsystems.Transfer_Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class Transfer_IntakeCommand_Reverse extends Command {    
    public Transfer_Intake m_intake;   
    public DoubleSupplier joystick;
    // private Timer timer;

    public Transfer_IntakeCommand_Reverse(Transfer_Intake intake) {
        this.m_intake = intake;
        addRequirements(m_intake);
        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
       m_intake.intake_init();
    }


    @Override   
    public void execute() {
        /* Get Values, Deadband*/
        //    m_intake.transferOn();
        m_intake.transferReverse();


    //    m_intake.transferReverse();

    }

    @Override
    public void end(boolean interrupted) {
        m_intake.transferOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}