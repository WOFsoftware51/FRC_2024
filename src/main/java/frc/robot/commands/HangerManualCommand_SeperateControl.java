package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hanger;


public class HangerManualCommand_SeperateControl extends Command {    
    public Hanger m_Hanger;   
    public DoubleSupplier joystick1;
    public DoubleSupplier joystick2;
    // private Timer timer;

    public HangerManualCommand_SeperateControl(Hanger hanger, DoubleSupplier joystickVal1, DoubleSupplier joystickVal2) {
        this.m_Hanger = hanger;
        this.joystick1 = joystickVal1;
        this.joystick2 = joystickVal2;
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
        double joystickFixed1 = MathUtil.applyDeadband(joystick1.getAsDouble(), Constants.stickDeadband);
        double joystickFixed2 = MathUtil.applyDeadband(joystick2.getAsDouble(), Constants.stickDeadband);
        m_Hanger.hangerSeperateControl(joystickFixed1, joystickFixed2);
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