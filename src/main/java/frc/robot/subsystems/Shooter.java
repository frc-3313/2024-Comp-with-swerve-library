// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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
  private RelativeEncoder feederEncoder = feederMotor.getEncoder();
  private static double maxSpeed = 1.0f;
  private ColorSensorV3 distanceSensor = new ColorSensorV3(Port.kMXP); 

  private SparkPIDController feederPID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double setDistance;

  public Shooter() 
  {
    shooterMotorOne.setIdleMode(IdleMode.kCoast);
    feederMotor.setIdleMode(IdleMode.kCoast);
    shooterMotorOne.setSmartCurrentLimit(80);
    feederMotor.setSmartCurrentLimit(40);
    feederMotor.setInverted(true);
    feederPID = feederMotor.getPIDController();
   // SmartDashboard.putBoolean("Display Shooter", displaySmartDashboard);


    //PID
    kP = 0.01; //how aggresive towards target
    kI = 0; //accumlation of past errors
    kD = 0.0005; //how rate of change responds
    kIz = 0;  //integral zone how much of the zone it looks at
    kFF = 0;  //feed forward again estimates the future based on past
    kMaxOutput = .5; //max motor speed
    kMinOutput = -.5; //max motor speed in oppisite direction 

    feederPID.setP(kP);
    feederPID.setI(kI);
    feederPID.setD(kD);
    feederPID.setIZone(kIz);
    feederPID.setFF(kFF);
    feederPID.setOutputRange(kMinOutput, kMaxOutput);
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

  public void MoveFeederDistance(double distance)
  {
    feederPID.setReference(feederEncoder.getPosition() + distance, ControlType.kPosition);
    setDistance = distance;
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

  
  public boolean FeederDone()
  {
    if(feederEncoder.getPosition() < setDistance + 3 &&  feederEncoder.getPosition() > setDistance - 3)
      return true;
    else
      return false;
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

    //  SmartDashboard.putNumber("DIo sensor", distanceSensor.getProximity());
    //  SmartDashboard.putNumber("shooter speed", shooterMotorOne.getEncoder().getVelocity());
      SmartDashboard.putNumber("shooter sensor", distanceSensor.getProximity());
    
    SmartDashboard.putBoolean("shooter has note", hasNote());
    SmartDashboard.putBoolean("shooter sensor connected", distanceSensor.isConnected());

  }
}
