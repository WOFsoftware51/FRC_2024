package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    public double speedMod = Constants.DRIVE_SPEED;
    private WPI_TalonSRX left_Drive = new WPI_TalonSRX(0);
    private WPI_TalonSRX left_Drive_Slave = new WPI_TalonSRX(0);
    
    private WPI_TalonSRX right_Drive = new WPI_TalonSRX(0);
    private WPI_TalonSRX right_Drive_Slave = new WPI_TalonSRX(0);
    private DifferentialDrive m_drive = new DifferentialDrive(right_Drive, left_Drive);
    



    public DriveTrain() {

    }

    public void drive_init(){
        left_Drive_Slave.follow(left_Drive);
    }

    public void drive(double translation, double rotation) {

    }    


  


    @Override
    public void periodic()
    {
        
    }
}