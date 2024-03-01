package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TelopSwerveAim extends Command {    
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private double rotationVal = 0;
    public double speedModifier = Constants.DRIVE_SPEED;
    private int isNegative = -1;


    public TelopSwerveAim(Swerve Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.s_Swerve = Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }
    

    @Override
    public void execute() {
      if(s_Swerve.tv == 1){

        rotationVal = s_Swerve.limelight_aim_proportional();
    }
    else{
        double yawFixed = s_Swerve.getGyroYaw().getDegrees()%360;
        SmartDashboard.putNumber("yawFixeds", yawFixed); 
        if(yawFixed<-30)
        {
        rotationVal = 0.3;
        }
        else if(yawFixed<-10)
        {
          rotationVal = 0.15;
        }
        else if(yawFixed<-1)
        {
          rotationVal = 0.05;
        }
        else if(yawFixed>30)  
        {
          rotationVal = -0.3;
        }
        else if(yawFixed>10)
        {
          rotationVal = -0.15;
        }
        else if(yawFixed>1)
        {
          rotationVal = -0.05;
        }
        else
        {
          rotationVal = 0.0;
        }
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