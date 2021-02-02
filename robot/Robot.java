// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot 
{
  // private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(0);
  // private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(1);
  // private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotor, rightMotor);

  // CAN Spark Max
  private final static CANSparkMax leftMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final static CANSparkMax rightMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

  // Talon SRX
  // private final static TalonSRX leftMotor = new TalonSRX(0);
  // private final static TalonSRX rightMotor = new TalonSRX(1);

  private final Joystick stick = new Joystick(0);
  
  public Robot()
  {
    // Talon SRX
    // rightMotor.follow(leftMotor);
  }

  @Override
  public void teleopInit()
  {
    System.out.println("Teleop Init\n");
  }

  @Override
  public void teleopPeriodic()
  {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    
    // m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());

    double speed = stick.getRawAxis(0) / 2.0;
    if(Math.abs(speed) > 0.2)
    {
      // CAN Spark Max
      leftMotor.set(speed);
      rightMotor.set(speed);

      // Talon SRX
      // leftMotor.set(ControlMode.PercentOutput, speed);
      // rightMotor.set(ControlMode.PercentOutput, speed);
    }
    else
    {
      // CAN Spark Max
      leftMotor.set(0.0);
      rightMotor.set(0.0);

      // Talon SRX
      // leftMotor.set(ControlMode.PercentOutput, 0.0);
      // rightMotor.set(ControlMode.PercentOutput, 0.0);
    }

    
  }
}
