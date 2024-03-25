package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Intake;


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
       Global_Variables.isIntaking = true;
    }


    @Override
    public void execute() {
        m_intake.intakeReverse();
    }


    @Override
    public void end(boolean interrupted) {
        m_intake.intakeOff();
        Global_Variables.isIntaking = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}