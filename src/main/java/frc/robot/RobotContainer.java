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

    private final SendableChooser<Double> s_chooser = new SendableChooser<>();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */    
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator m_Elevator = new Elevator();
    private final Hanger m_Hanger = new Hanger();
    private final Intake m_Intake = new Intake();
    private final Turret m_Turret = new Turret();
    private final Shooter m_Shooter = new Shooter();



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        SmartDashboard.putData("Auton", a_chooser);
        a_chooser.setDefaultOption("Test Auto", 1);
        a_chooser.addOption("Example Auto", 2);



        SmartDashboard.putData("Shot Speed", s_chooser);

        s_chooser.addOption("1000 rpm", 1000.0);
        s_chooser.setDefaultOption("4000 rpm",  4000.0);
        for(int i = 1; i < 20; i++){
            s_chooser.addOption(4000 + i*500 + " rpm",  4000.0 + i*500);
        }






        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false //() -> robotCentric.getAsBoolean()
            )
        );
        
        m_Turret.setDefaultCommand(new TurretCommand(m_Turret, ()-> operator.getLeftY()));

        m_Hanger.setDefaultCommand(new HangerManualCommand(m_Hanger, ()-> operator.getRightY()));

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
        new Trigger(() -> driver.getRightTriggerAxis() >0.8).whileTrue(new Right_Trigger_Boost_True());

        new Trigger(operator::getBackButton).whileTrue(new ElevatorCommand(m_Elevator, true));
        new Trigger(operator::getStartButton).whileTrue(new ElevatorCommand(m_Elevator, false));

        new Trigger(operator::getLeftBumper).whileTrue(new IntakeCommand(m_Intake));
        new Trigger(operator::getRightBumper).whileTrue(new IntakeCommand_Reverse(m_Intake));

        new Trigger(operator::getAButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.A_Button));
        new Trigger(operator::getBButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.B_Button));
        new Trigger(operator::getXButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.X_Button));
        new Trigger(operator::getYButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.Y_Button));

        new Trigger(operator::getBackButton).whileTrue(new HangerCommand(m_Hanger, true));
        new Trigger(operator::getStartButton).whileTrue(new HangerCommand(m_Hanger, false));



        new Trigger(() -> operator.getRightTriggerAxis() > 0.80).whileTrue(new IntakeCommand_Reverse(m_Intake));

        new Trigger(() -> operator.getLeftTriggerAxis() > 0.80).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));

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