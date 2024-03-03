package frc.robot.commands;

import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand_Reverse extends Command {    
    public Intake m_intake;   
    public DoubleSupplier joystick;
    // private Timer timer;

    public IntakeCommand_Reverse(Intake intake) {
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
       m_intake.intakeReverse();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}