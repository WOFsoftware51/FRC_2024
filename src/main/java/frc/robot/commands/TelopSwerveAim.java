package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TelopSwerveAim extends Command {    
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private double rotationVal = 0;
    private double speedModifier = Constants.DRIVE_SPEED;


    public TelopSwerveAim(Swerve Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.s_Swerve = Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }
    

    @Override
    public void execute() {
    if(Global_Variables.tv == 1){
      rotationVal = s_Swerve.limelight_aim_proportional();
    }
    else{
      // rotationVal = s_Swerve.gotoDefaultGyroVal();
    }
    

    if(Global_Variables.left_trigger_boost)
    {
        speedModifier = 0.1;   
    }
    else if(Global_Variables.right_trigger_boost)
    {
        speedModifier = 1.0;
    }
    else
    {
        speedModifier = Constants.DRIVE_SPEED;
    }
    
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedModifier), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        false, 
        true
    );
    }
}