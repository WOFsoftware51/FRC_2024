package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Elevator extends SubsystemBase {


  private TalonFX _elevator = new TalonFX(Constants.elevator, Constants.CANIVORE_NAME); //Constants.elevator2
  private TalonFX _elevator2 = new TalonFX(Constants.elevator2, Constants.CANIVORE_NAME);
  private DigitalInput limitSwitch = new DigitalInput(8);

  private MotionMagicDutyCycle mMDutyCycle = new MotionMagicDutyCycle(0);

  private int count = 0;

  /** Creates a new extend. */
  public void elevator_init()
  {

    if(count< 1)
    {
      elevator_resetEncoder();
      count++;
    }

    // _elevator.setNeutralMode(NeutralModeValue.Coast);
    // _elevator.setNeutralMode(NeutralModeValue.Coast);
    _elevator.setInverted(true);
    _elevator2.setInverted(false);

    // TalonFXConfiguration cfg = new TalonFXConfiguration();
    // MotionMagicConfigs mm = cfg.MotionMagic;
    // mm.MotionMagicCruiseVelocity = 80; // 5 rotations per second cruise
    // mm.MotionMagicAcceleration = 160; //80 // Take approximately 0.5 seconds to reach max vel
    // mm.MotionMagicJerk = 800;// Take approximately 0.2 seconds to reach max accel 

    // Slot0Configs slot0 = cfg.Slot0;
    // slot0.kP = 0.060058651; //5
    // slot0.kI = 0;
    // slot0.kD = 0.0; //0.1
    // slot0.kV = 0.0; //0.12
    // slot0.kS = 0.0; //0.25 // Approximately 0.25V to get the mechanism moving
    // mMDutyCycle.withFeedForward(0.07);


    // _elevator.getConfigurator().apply(cfg, 0.050);


    // FeedbackConfigs fConfigs = cfg.Feedback;
    // fConfigs.SensorToMechanismRatio = 70;


    // cfg.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    // cfg.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 8;
    // cfg.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
    // cfg.HardwareLimitSwitch.withForwardLimitAutosetPositionValue(1);
    // // cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.411;
    // cfg.HardwareLimitSwitch.withForwardLimitAutosetPositionEnable(true);
    // // cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    // cfg.HardwareLimitSwitch.withForwardLimitEnable(true);

    // cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -80;

    // _elevator.getConfigurator().apply(cfg, 0.050);


    _elevator.setNeutralMode(NeutralModeValue.Brake);
    _elevator2.setNeutralMode(NeutralModeValue.Brake);
    // _elevator.setInverted(false);

    

    _elevator2.setControl(new Follower(Constants.elevator, true));

  }

  public void elevator_resetEncoder()
  {
    _elevator.setPosition(0);
  }

  // public void elevator_Goto_Position(double value)
  // {
  //   mMDutyCycle.Slot = 0;
  //   _elevator.setControl(mMDutyCycle.withPosition(value/360));  //Don't know the position right now  //Divide by 360 to convert to Degrees
  // }



  public void elevatorOn(){
    _elevator.set((-0.5));
  }

    public void elevatorOn(double x){
    // _elevator.setControl(voltageOut.withOutput(12*(x*0.5)));
    _elevator.set(-x*0.85);
  }

  public void elevatorReverse(){
    _elevator.set(-0.4);
  }

  public void elevatorOff(){
    _elevator.set(0);
  }


  public double elevator_speed()
  {
    double elevator_speed = _elevator.getVelocity().getValue();
    return elevator_speed;
  }

  public double elevator_encoder()
  {
    double elevator_encoder = _elevator.getPosition().getValue();
    return elevator_encoder;
  }

  public int limitSwitchVal(){
    if(limitSwitch.get()){
      return 1;
    }
    else{
      return -1;
    }
}

  public void stopMotor(){;
    _elevator.stopMotor();
  }

  public void getMotorPow(){
    _elevator.getAcceleration();
  }

  // public double getClosedLoopOutput(){
  //   return _elevator.getClosedLoopOutput().getValue();
  // }
  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder", elevator_encoder());
    SmartDashboard.putNumber("Elevator LimitSwitch", limitSwitchVal());
    // SmartDashboard.putNumber("Elevator Closed Loop Output", _elevator.getClosedLoopOutput().getValue());

    if(limitSwitchVal() == -1){
      elevator_resetEncoder();
    }

  }
}