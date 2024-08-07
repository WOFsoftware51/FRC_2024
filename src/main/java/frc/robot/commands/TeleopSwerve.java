package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    public double speedModifier = Constants.DRIVE_SPEED;
    double translationVal = 0;//MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = 0;//MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = 0;//MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    public TeleopSwerve(Swerve Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }
    

    @Override
    public void execute() {

        
        
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

        /* Get Values, Deadband*/
        translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedModifier), 
            rotationVal * Constants.Swerve.maxAngularVelocity*speedModifier, 
            !robotCentricSup.getAsBoolean(), 
            true
        );


    }
}