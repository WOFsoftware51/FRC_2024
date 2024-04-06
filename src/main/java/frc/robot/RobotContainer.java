package frc.robot;

import java.util.Optional;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Shoot_Only_Auto;
import frc.robot.autos.TestAuton;
import frc.robot.autos.Blue_Autos.Blue_Leave_Zone;
import frc.robot.autos.Blue_Autos.Blue_Leave_Zone_0;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_0_3;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_0_3_2;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_0_3_2_1;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_Bumper_0_3;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_Bumper_0_7_Far;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_Bumper_0_8_Far;
import frc.robot.autos.Blue_Autos.Middle_Autos.Blue_Auto_Bumper_0_2_1_Far;
import frc.robot.autos.Blue_Autos.Middle_Autos.Blue_Auto_Middle_Bumper_0_2_3;
import frc.robot.autos.Blue_Autos.Middle_Autos.Blue_Auto_Middle_Bumper_0_3;
import frc.robot.autos.Blue_Autos.Bottom_Autos.Blue_Auto_Bumper_0_8_7_Far;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_0_1;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_0_1_2;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_0_1_2_3;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_Bumper_0_1_2;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_Bumper_0_1_4;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_Bumper_0_1_4_5_Far;
import frc.robot.autos.Blue_Autos.Top_Autos.Blue_Auto_Bumper_0_1_5_Far;
import frc.robot.autos.Red_Autos.Red_Leave_Zone;
import frc.robot.autos.Red_Autos.Red_Leave_Zone_0;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_0_3;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_0_3_2;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_0_3_2_1;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_Bumper_0_3;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_Bumper_0_7_Far;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_Bumper_0_8_Far;
import frc.robot.autos.Red_Autos.Bottom_Autos.Red_Auto_Bumper_0_8_7_Far;
import frc.robot.autos.Red_Autos.Middle_Autos.Red_Auto_Bumper_0_2_1_Far;
import frc.robot.autos.Red_Autos.Middle_Autos.Red_Auto_Middle_0_2;
import frc.robot.autos.Red_Autos.Middle_Autos.Red_Auto_Middle_0_2_Far;
import frc.robot.autos.Red_Autos.Middle_Autos.Red_Auto_Middle_Bumper_0_2_3;
import frc.robot.autos.Red_Autos.Middle_Autos.Red_Auto_Middle_Bumper_0_3;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_0_1;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_0_1_2;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_0_1_2_3;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_Bumper_0_1;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_Bumper_0_1_2;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_Bumper_0_1_4;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_Bumper_0_1_4_5_Far;
import frc.robot.autos.Red_Autos.Top_Autos.Red_Auto_Bumper_0_1_5_Far;
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
    private final XboxController testController = new XboxController(2);
    private final SendableChooser<Integer> a_chooser = new SendableChooser<>();
    // private final SendableChooser<Boolean> b_chooser = new SendableChooser<>();


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
    private final Transfer_Intake m_Transfer = new Transfer_Intake();
    private final Turret m_Turret = new Turret();
    private final Shooter m_Shooter = new Shooter();
    private final Auton_Subsystem m_aSub = new Auton_Subsystem();
    private final CANdle_Subsystem m_Candle = new CANdle_Subsystem();
    




    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        printAutons();

        // SmartDashboard.putData("Auton", a_chooser);
        // a_chooser.setDefaultOption("Test Auto", 1);
        // a_chooser.addOption("Example Auto", 2);

        // SmartDashboard.putData("Turret Manual Enable", b_chooser);
        // b_chooser.setDefaultOption("Hangar Manual Controller", false);
        // b_chooser.addOption("Turret Manual Controller", true);

        

        SmartDashboard.putData("Shot Speed", s_chooser);

        s_chooser.addOption("1000 rpm", 1000.0);
        s_chooser.addOption("2000 rpm",  2000.0);
        s_chooser.addOption("3000 rpm",  3000.0);
        s_chooser.addOption("3500 rpm",  3500.0);
        s_chooser.setDefaultOption("4000 rpm",  4000.0);


        // SmartDashboard.putData("Limelight Pipeline Chooser", Global_Variables.pipeline_chooser);

        // s_chooser.setDefaultOption("Don't Overide Limelight Control",  false);
        // s_chooser.addOption("Overide Limelight Control",  true);





        s_Swerve.setDefaultCommand(
            new TeleopSwerve2(// new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false, //() -> robotCentric.getAsBoolean(),
                ()-> operator.getXButton()
            )
        );
        

        m_Elevator.setDefaultCommand(new ElevatorCommand(m_Elevator, ()-> -operator.getLeftY())); //-operator.getLeftY()));
        m_Turret.setDefaultCommand(new TurretCommand(m_Turret, ()-> testController.getRightY()));
        m_Hanger.setDefaultCommand(new HangerManualCommand(m_Hanger, ()-> -operator.getRightY()));

        // m_Hanger.setDefaultCommand(new HangerManualCommand_SeperateControl(m_Hanger, ()-> testController.getRightY(), ()-> testController.getLeftY()));


        m_Candle.setDefaultCommand(new CANdle_Default(m_Candle));

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
        // new Trigger((() -> operator.getRightTriggerAxis() > 0.80)).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));



        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /* Driver Buttons */
    //    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroDrive()));
        // zeroGyro.onTrue(new RunCommand(() -> s_Swerve.zeroDrive()));
        // new Trigger(driver::getAButton).onTrue(new InstantCommand(()-> s_Swerve.zeroDrive()));
        new Trigger(() -> driver.getRightTriggerAxis() >0.8).whileTrue(new Right_Trigger_Boost_True());
        new Trigger(driver::getBButton).whileTrue(new Transfer_IntakeCommand_No_Sensor(m_Transfer)); //What this???
        new Trigger(testController::getXButton).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));

        
        

        /*CANDle Commands*/
        // new Trigger(() -> driver.getRightTriggerAxis() >0.8).whileTrue(new CANdle_Solid_White_Animation(m_Candle));
        // new Trigger(driver::getLeftBumper).whileTrue(new CANdle_Purple_Command(m_Candle)); 
        new Trigger((() -> operator.getLeftTriggerAxis() > 0.80)).whileTrue(new CANdle_Intake_Command(m_Candle));

        // new Trigger(testController::getBButton).whileTrue(new RunCommand(()-> m_Candle.CANdle_Solid_Green()));
        // new Trigger(testController::getAButton).whileTrue(new RunCommand(()-> m_Candle.CANdle_Red()));
        // new Trigger(driver::getRightBumper).whileTrue(new CANdle_LockOn_Command(m_Candle));
        new Trigger(operator::getXButton).whileTrue(new CANdle_LockOn_Command(m_Candle));
        // new Trigger(drivser::getLeftBumper).whileTrue(new CANdle_LockOn_Command(m_Candle));
        new Trigger(driver::getBButton).whileTrue(new CANdle_LockOn_Command(m_Candle));



        /*Aiming to Score and Rev up Shooter: Driver[RightBumper] */
        // new Trigger(operator::getXButton).whileTrue(new TelopSwerveAim(s_Swerve, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis)));
        new Trigger(operator::getXButton).whileTrue(new TurretAim(m_Turret));
        new Trigger(operator::getXButton).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
        new Trigger(driver::getLeftBumper).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
        new Trigger(driver::getLeftBumper).whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_DEFAULT_POSITION));

        /**Half Court Shot */
        new Trigger(driver::getRightBumper).whileTrue(new ShootCommand(m_Shooter, ()-> 2500));
        new Trigger(driver::getRightBumper).whileTrue(new Turret_Goto_Angle(m_Turret, 12.5));

        new Trigger(driver::getBackButton).whileTrue(new ShootCommand_Top(m_Shooter, ()-> 0.3));
        new Trigger(driver::getBackButton).whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_DEFAULT_POSITION+5));


        /*GOTO Default Position and Rev up Shooter: Driver[LeftBumper]*/
        // new Trigger(driver::getLeftBumper).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
        new Trigger(operator::getBButton).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));

        // new Trigger(driver::getLeftBumper).whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_PROTECTED_POSITION));
        new Trigger(operator::getBButton).whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_PROTECTED_POSITION));

        /*Transfer and Floor Intake: Operator[LeftTrigger]*/
        new Trigger((() -> operator.getLeftTriggerAxis() > 0.80)).whileTrue(new IntakeCommand(m_Intake));
        new Trigger((() -> operator.getLeftTriggerAxis() > 0.80)).whileTrue(new Transfer_IntakeCommand(m_Transfer));
        // new Trigger((() -> operator.getLeftTriggerAxis() > 0.80)).whileTrue(new RunCommand(()-> driverRumble()));

        // new Trigger((() -> operator.getLeftTriggerAxis() > 0.80)).whileTrue(new Transfer_IntakeShoot(m_Transfer));

        /*Transfer Intake Poop: Operator[RightBumper]*/
        new Trigger(operator::getRightBumper).whileTrue(new Transfer_IntakeCommand_Reverse(m_Transfer));

        /*Floor Intake Reverse: Operator[LeftBumper] */
        new Trigger(operator::getLeftBumper).whileTrue(new IntakeCommand_Reverse(m_Intake));

        /*Elevator Setpositions: Operator[A, B, X, Y]*/
        new Trigger(operator::getAButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.A_Button));//////
        // new Trigger(operator::getBButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.B_Button);
        // new Trigger(operator::getXButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.X_Button));
        new Trigger(operator::getYButton).whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.Y_Button));//////

        /* Hangar Command: Operator[BackButton, StartButton]*/
        new Trigger(operator::getBackButton).whileTrue(new HangerCommand(m_Hanger, true));
        new Trigger(operator::getStartButton).whileTrue(new HangerCommand(m_Hanger, false));

        /*Shoot: Operator[RightTrigger]*/

        // new Trigger((() -> operator.getRightTriggerAxis() > 0.80)).whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
        new Trigger((() -> operator.getRightTriggerAxis() > 0.80)).whileTrue(new Transfer_IntakeShoot(m_Transfer));

    }

    public void driverRumble(){
        if(Global_Variables.getSensorVal()==(1)){
            driver.setRumble(RumbleType.kBothRumble, 1);
        }
        else{
            driver.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public void printAutons(){
        SmartDashboard.putData("Auton", a_chooser);

        a_chooser.setDefaultOption("Leave Zone", 1);
        // a_chooser.addOption("Leave Zone + Score", 2);

        // a_chooser.addOption("0, 1, Auto", 3);
        // a_chooser.addOption("0, 1, 2 Auto", 4);
        // a_chooser.addOption("0, 1, 2, 3 Auto", 5);
        // a_chooser.addOption("0, 3, Auto", 6);
        // a_chooser.addOption("0, 3, 2 Auto", 7);
        // a_chooser.addOption("0, 3, 2, 1 Auto", 8);
        a_chooser.addOption("0, 2 Bumper Auto", 9);
        a_chooser.addOption("0, 3 Bumper Auto", 10);
        a_chooser.addOption("0, 1 Bumper Auto", 11);
        a_chooser.addOption("0, 8 Bumper Auto", 12);
        // a_chooser.addOption("0, 8, 7 Bumper Auto", 13);
        a_chooser.addOption("0, 1, 4 Bumper Auto", 14);
        a_chooser.addOption("Test Auto", 15);
        a_chooser.addOption("0, 2 Bumper Far Auto", 16);
        a_chooser.addOption("Shoot Only Auto", 17);
        a_chooser.addOption("0, 7 Bumper Far Auto", 18);
        a_chooser.addOption("0, 1, 5 Bumper Far Auto", 19);
        a_chooser.addOption("0, 7, 8 Bumper Far Auto", 20);
        a_chooser.addOption("0, 1, 4, 5 Bumper Far Auto", 21);
        a_chooser.addOption("0, 2, 1 Bumper Far Auto", 22);
        a_chooser.addOption("0, 2, 3 Middle Bumper Auto", 23);
        a_chooser.addOption("0, 3 Middle Bumper Auto", 24);

    }        


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        Command auton = null;
        // An ExampleCommand will run in autonomous

        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                auton = redAutons();
            }
            if (ally.get() == Alliance.Blue) {
                auton = blueAutons();
            }
        }
        else {
            auton = blueAutons();
        }
        return auton;

    }


    private Command blueAutons(){
        switch (a_chooser.getSelected()) 
        {
            case 1: return new Blue_Leave_Zone(s_Swerve);
            case 2: return new Blue_Leave_Zone_0(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 3: return new Blue_Auto_0_1(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 4: return new Blue_Auto_0_1_2(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 5: return new Blue_Auto_0_1_2_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 6: return new Blue_Auto_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 7: return new Blue_Auto_0_3_2(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 8: return new Blue_Auto_0_3_2_1(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 9: return new Red_Auto_Middle_0_2(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 10: return new Blue_Auto_Bumper_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 11: return new Red_Auto_Bumper_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 12: return new Blue_Auto_Bumper_0_8_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 13: return new Blue_Auto_Bumper_0_8_7_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 14: return new Blue_Auto_Bumper_0_1_4(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 15: return new TestAuton(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake); //TestAuton
            case 16: return new Red_Auto_Middle_0_2_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 17: return new Shoot_Only_Auto(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 18: return new Blue_Auto_Bumper_0_7_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 19: return new Blue_Auto_Bumper_0_1_5_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 20: return new Blue_Auto_Bumper_0_8_7_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 21: return new Blue_Auto_Bumper_0_1_4_5_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 22: return new Blue_Auto_Bumper_0_2_1_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 23: return new Blue_Auto_Middle_Bumper_0_2_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 24: return new Blue_Auto_Middle_Bumper_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);

            default: return new Shoot_Only_Auto(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        }
    }

    private Command redAutons(){
        switch (a_chooser.getSelected()) 
        {
            case 1: return new Red_Leave_Zone(s_Swerve);
            case 2: return new Red_Leave_Zone_0(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 3: return new Red_Auto_0_1(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 4: return new Red_Auto_0_1_2(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 5: return new Red_Auto_0_1_2_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 6: return new Red_Auto_0_3_2_1(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 7: return new Red_Auto_0_3_2(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 8: return new Red_Auto_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 9: return new Red_Auto_Middle_0_2(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 10: return new Red_Auto_Bumper_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 11: return new Red_Auto_Bumper_0_1(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 12: return new Red_Auto_Bumper_0_8_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 13: return new Red_Auto_Bumper_0_8_7_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 14: return new Red_Auto_Bumper_0_1_4(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 15: return new TestAuton(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);  ///////////////// TestAuton
            case 16: return new Red_Auto_Middle_0_2_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 17: return new Shoot_Only_Auto(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 18: return new Red_Auto_Bumper_0_7_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 19: return new Red_Auto_Bumper_0_1_5_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 20: return new Red_Auto_Bumper_0_8_7_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 21: return new Red_Auto_Bumper_0_1_4_5_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 22: return new Red_Auto_Bumper_0_2_1_Far(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 23: return new Red_Auto_Middle_Bumper_0_2_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
            case 24: return new Red_Auto_Middle_Bumper_0_3(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);

            default: return new Shoot_Only_Auto(s_Swerve, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        }
    }
}

