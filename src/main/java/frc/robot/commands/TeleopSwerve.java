package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.DriveTrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private DriveTrain m_DriveTrain;
    private DoubleSupplier translationSup;
    private DoubleSupplier rotationSup;
    public double speedModifier = Constants.DRIVE_SPEED;


    public TeleopSwerve(DriveTrain driveTrain, DoubleSupplier translationSup, DoubleSupplier rotationSup) {
        this.m_DriveTrain = driveTrain;
        addRequirements(m_DriveTrain);

        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
    }
    

    @Override
    public void execute() {
        
        if(Global_Variables.left_bumper_boost)
        {
            speedModifier = 0.1;   
        }
        else if(Global_Variables.right_bumper_boost)
        {
            speedModifier = 1.0;
        }
        else
        {
            speedModifier = Constants.DRIVE_SPEED;
        }
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        m_DriveTrain.drive(
            translationVal, 
            rotationVal
        );
    }
}