package frc.robot.commands_Auton;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class AutonSwerveAim extends Command {    
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    public boolean endCommand = false;
    public int count = 0;


    private double rotationVal = 0;
    private double speedModifier = Constants.DRIVE_SPEED;


    public AutonSwerveAim(Swerve Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.s_Swerve = Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }
    @Override
    public void initialize() {
      /* Get Values, Deadband*/
      endCommand = false;
      count = 0;
      speedModifier = Constants.DRIVE_SPEED;
  }


    @Override
    public void execute() {

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

    if(Global_Variables.tv == 1){

      if(Global_Variables.tx < 4.0 && Global_Variables.tx > -4.0 && count > 10){
        endCommand = true;
        rotationVal = 0.0;
      }
      else{
        rotationVal = s_Swerve.limelight_aim_proportional();
      }
      count++;
    }
    else{
      // rotationVal = s_Swerve.gotoDefaultGyroVal();
    }
    
    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedModifier), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        false, 
        true
    );
    }

    
    @Override
    public void end(boolean interrupted) {
      s_Swerve.drive(
        new Translation2d(0, 0), 
          0, 
          false, 
          true
      );

    }

    @Override
    public boolean isFinished() {
        return endCommand;
    }

}