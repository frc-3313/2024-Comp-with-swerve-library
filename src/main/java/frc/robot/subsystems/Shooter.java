// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase 
{

  private final CANSparkMax shooterMotorOne = new CANSparkMax((Constants.Shooter.ShooterMotor1_ID), MotorType.kBrushless);
  private final CANSparkMax feederMotor = new CANSparkMax(Constants.Shooter.FeederMotor_ID, MotorType.kBrushless);
  private static double maxSpeed = 1.0f;
  private ColorSensorV3 distanceSensor = new ColorSensorV3(Port.kMXP); 
  private boolean displaySmartDashboard;

  public Shooter() 
  {
    shooterMotorOne.setIdleMode(IdleMode.kCoast);
    feederMotor.setIdleMode(IdleMode.kCoast);
    shooterMotorOne.setSmartCurrentLimit(80);
    feederMotor.setSmartCurrentLimit(40);
    feederMotor.setInverted(true);
    SmartDashboard.putBoolean("Display Shooter", displaySmartDashboard);
  }

  public void StartShooter()
  {
    shooterMotorOne.set(maxSpeed);
  }
 public void StartShooter(double speed)
  {
    shooterMotorOne.set(speed);
  }
  public void StartFeeder(double speed)
  {
    feederMotor.set(speed);
  }

  public void StartFeeder()
  {
    feederMotor.set(maxSpeed);
  }

  public void StopShooter()
  {
    shooterMotorOne.set(0);
  }

  public void StopFeeder()
  {
    feederMotor.set(0);
  }
  public void StopAllMotors()
  {
    shooterMotorOne.set(0);
    feederMotor.set(0);
  }

  public boolean hasNote()
  {
    if(GetDistance() > 50 )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

   public int GetDistance()
   {
      return distanceSensor.getProximity();
   }

  public boolean IsShooterAboveRPM(int rpm)
  {
    if(shooterMotorOne.getEncoder().getVelocity() > rpm)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  @Override
  public void periodic() 
  {
    displaySmartDashboard = SmartDashboard.getBoolean("Display Elevator", displaySmartDashboard);
    if(displaySmartDashboard)
    {
      SmartDashboard.putNumber("DIo sensor", distanceSensor.getProximity());
      SmartDashboard.putNumber("shooter speed", shooterMotorOne.getEncoder().getVelocity());
      SmartDashboard.putNumber("shooter sensor", distanceSensor.getProximity());
      SmartDashboard.putBoolean("shooter has note", hasNote());
    }
  }
}
