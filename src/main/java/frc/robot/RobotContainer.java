package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);
    private final SendableChooser<Integer> a_chooser = new SendableChooser<>();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */    
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm m_Arm = new Arm();
    /* Music */
    private final String[] sList =
    {
        //Add songs to here
        "Beethoven-Moonlight-Sonata.chrp"

    }; 




    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {



    SmartDashboard.putData("Auton", a_chooser);
    
    a_chooser.setDefaultOption("Test Auto", 1);
    a_chooser.addOption("Example Auto", 2);



    SmartDashboard.putData("Songs", Global_Variables.song);


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false //() -> robotCentric.getAsBoolean()
            )
        );
        m_Arm.setDefaultCommand(new Arm_Command(m_Arm, () -> operator.getLeftY()));


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new Trigger(driver::getRightBumper).whileTrue(new TelopSwerveAim(s_Swerve, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis)));


        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        new Trigger(driver::getRightBumper).whileTrue(new TelopSwerveAim(s_Swerve, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis)));

        // new Trigger(drover::getRightBumper).whileFalse(new Right_Bumper_Boost_False());
        new Trigger(driver::getRightBumper).whileTrue(new Right_Trigger_Boost_True());
        new Trigger(driver::getBackButton).whileTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        // new Trigger(operator::getAButton).whileTrue(new Music(s_Swerve)); 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        switch (a_chooser.getSelected()) 
        {
        case 1: return new Square_Auto(s_Swerve);
        case 2: return new exampleAuto();

        default: return new Square_Auto(s_Swerve);
        }
    }
}