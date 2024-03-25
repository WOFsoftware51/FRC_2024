package frc.robot.commands;

import frc.robot.Global_Variables;
import frc.robot.subsystems.Transfer_Intake;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class Transfer_IntakeCommand_No_Sensor extends Command {    
    public Transfer_Intake m_transferIntake;   
    public DoubleSupplier joystick;
    // private Timer timer;

    public Transfer_IntakeCommand_No_Sensor(Transfer_Intake transferIntake) {
        this.m_transferIntake = transferIntake;
        addRequirements(m_transferIntake);
        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
        m_transferIntake.intake_init();
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/
        m_transferIntake.transferOn();
    }

    @Override
    public void end(boolean interrupted) {
        m_transferIntake.transferOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}