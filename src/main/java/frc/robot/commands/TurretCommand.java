package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;


public class TurretCommand extends Command {    
    public Turret m_turret;   
    public DoubleSupplier joystick;
    // private Timer timer;

    public TurretCommand(Turret turret, DoubleSupplier _joystick) {
        this.m_turret = turret;
        addRequirements(m_turret);
        this.joystick = _joystick;
        // timer = new Timer();
    }

    @Override
    public void initialize() {
        /* Get Values, Deadband*/
       m_turret.turret_init();
    }


    @Override
    public void execute() {
        double joystickFixed = MathUtil.applyDeadband(joystick.getAsDouble(), Constants.stickDeadband);
        m_turret.turretOn(joystickFixed);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turretOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}