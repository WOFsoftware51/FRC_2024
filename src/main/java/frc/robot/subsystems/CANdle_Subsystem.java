// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.*;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;



public class CANdle_Subsystem extends SubsystemBase 
{
 
   /** Creates a new CANdle. */
  private final CANdle m_candle = new CANdle(0, Constants.CANIVORE_NAME);
  private final int LedCount = 70;
  private Animation m_toAnimate;

  public void CANdle_init() 
  {  
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
     //   configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
  }
 
  public void CANdle_off() 
  {  
    m_toAnimate = new StrobeAnimation(0, 0, 0, 0, 0, LedCount, 9);
  }
  
  public void CANdle_Animate() 
  {  
    m_candle.animate(m_toAnimate);
  }
  /** Flashing Purple*/
  public void CANdle_Purple() 
  {  
    m_toAnimate = new StrobeAnimation(100, 10, 100, 0, 0.3, LedCount, 9);
  }

  public void CANdle_Purple_Blink() 
  {  
    m_toAnimate = new StrobeAnimation(100, 10, 100, 0, 0.1, LedCount, 9);
  }

  public void CANdle_Orange() 
  {  
    m_toAnimate = new StrobeAnimation(255, 150 , 5, 0, 0.3, LedCount, 9);
  }

  /** Not Flashing Purple*/
  public void CANdle_Default() 
  {  
    //m_toAnimate = new StrobeAnimation(100, 10, 100, 0, 1.0, LedCount, 9);
    m_toAnimate = new RainbowAnimation(1.0, 0.1, LedCount, false, 0);
  }

  public void CANdle_Rainbow() 
  {  
    m_toAnimate = new RainbowAnimation(1.0, 1.0, LedCount, false, 9);
  }

  public void CANdle_Solid_Green() 
  {  
    m_toAnimate = new StrobeAnimation(10, 255, 10, 0, 1.0, LedCount, 9);
  }
  public void CANdle_Red() 
  {  
    m_toAnimate = new StrobeAnimation(100, 0, 0, 0, 1, LedCount, 1);
  }

  public void CANdle_Blue() 
  {  
    m_toAnimate = new StrobeAnimation(0, 0, 100, 0, 1, LedCount, 6);
  }

  public void CANdle_Purple_Larson() 
  {  
    m_toAnimate = new LarsonAnimation(100, 10, 100, 0, 1.0, LedCount, BounceMode.Front, 7, 9);
  }

  @Override
  public void periodic() 
  {  
    m_candle.animate(m_toAnimate);
  }
}

