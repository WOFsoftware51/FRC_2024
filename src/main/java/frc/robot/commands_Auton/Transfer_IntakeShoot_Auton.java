package frc.robot.commands_Auton;

import frc.robot.Global_Variables;
import frc.robot.subsystems.Transfer_Intake;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class Transfer_IntakeShoot_Auton extends Command {    
    public Transfer_Intake m_transferIntake;   
    public DoubleSupplier joystick;
    // private Timer timer;

    public Transfer_IntakeShoot_Auton(Transfer_Intake transferIntake) {
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
        m_transferIntake.transferOn();
        m_transferIntake.shooter_transferReverse(); 
    }

    @Override
    public void end(boolean interrupted) {
        m_transferIntake.transferOff();
        m_transferIntake.shooter_transferOff(); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}