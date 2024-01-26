package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final String CANIVORE_NAME = "CANivore";
    public static final double DRIVE_SPEED = 0.6;




    public static final class DriveTrain {


    }
    

    public static int shooter = 44;
    public static int turret = 00;
    public static int shooter2 = 45;
    public static int intake = 46;

	public static double shootP = 0.0;
	public static double shootI= 0.0;
	public static double shootD= 0.0;
	public static double shootF= 0.08;//6;
}
