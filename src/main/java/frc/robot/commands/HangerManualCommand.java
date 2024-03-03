package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hanger;


public class HangerManualCommand extends Command {    
    public Hanger m_Hanger;   
    public DoubleSupplier joystick;
    // private Timer timer;

    public HangerManualCommand(Hanger hanger, DoubleSupplier joystickVal) {
        this.m_Hanger = hanger;
        this.joystick = joystickVal;
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
        double joystickFixed = MathUtil.applyDeadband(joystick.getAsDouble(), Constants.stickDeadband);
        m_Hanger.hangerJoystick(joystickFixed);
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