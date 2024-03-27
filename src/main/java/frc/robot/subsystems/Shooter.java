// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase 
{
  private final CANSparkMax shooterMotorOne = new CANSparkMax((Constants.Shooter.ShooterMotor1_ID), MotorType.kBrushless);
  private final CANSparkMax feederMotor = new CANSparkMax(Constants.Shooter.FeederMotor_ID, MotorType.kBrushless);
  private RelativeEncoder feederEncoder = feederMotor.getEncoder();
  private static double maxSpeed = 1.0f;
  private DigitalInput shootToCloseBeam = new DigitalInput(0);
  private DigitalInput shootHasNoteBeam = new DigitalInput(1);
  private SparkPIDController feederPID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double setDistance;
  private double estRPM;
  private boolean feederStarted;

  public Shooter() 
  {
    shooterMotorOne.setIdleMode(IdleMode.kCoast);
    feederMotor.setIdleMode(IdleMode.kCoast);
    shooterMotorOne.setSmartCurrentLimit(80);
    feederMotor.setSmartCurrentLimit(40);
    feederMotor.setInverted(true);
    feederPID = feederMotor.getPIDController();
    feederStarted = false;
    
    //PID
    kP = 0.02; //how aggresive towards target
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


 public void SetShooterSpeed(double speed)
  {
    shooterMotorOne.set(speed);
    estRPM = ((1700/.3) * speed) -300;
  }
  
  public void StartFeeder(double speed)
  {
    feederStarted = false;
    feederPID.setReference(speed, ControlType.kVelocity);
  }

  public void StartFeeder()
  {
    feederStarted = false;
    feederPID.setReference(maxSpeed, ControlType.kVelocity);
  }

  public void StopShooter()
  {
    feederStarted = false;
    shooterMotorOne.set(0);
  }

  public void StopFeeder()
  {
    feederStarted = false;
    feederPID.setReference(0, ControlType.kVelocity);
  }
  public void StopAllMotors()
  {
    feederStarted = false;
    shooterMotorOne.set(0);
    feederPID.setReference(0, ControlType.kVelocity);
  }

  public void MoveFeederDistance(double distance)
  {
    feederStarted = true;
    setDistance = feederEncoder.getPosition() + distance;
    feederPID.setReference(setDistance, ControlType.kPosition);
  }

  public boolean feederStarted()
  {
    return feederStarted;
  }

  public boolean hasNote()
  {

    return !shootHasNoteBeam.get();

  }
   
   public Boolean noteToClose()
   {
      return !shootToCloseBeam.get();
   }

  
  public boolean FeederDone()
  {
    if(feederEncoder.getPosition() < setDistance + 3 &&  feederEncoder.getPosition() > setDistance - 3)
      return true;
    else
      return false;
  }
  public boolean IsShooterAboveRPM()
  {
    if(shooterMotorOne.getEncoder().getVelocity() > estRPM)
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

    SmartDashboard.putBoolean("shooter has note", noteToClose());
    SmartDashboard.putBoolean("Note To Close", hasNote());    
    SmartDashboard.putNumber("ShooterRPM", shooterMotorOne.getEncoder().getVelocity());
    SmartDashboard.putNumber("CurrentDistance", feederEncoder.getPosition());
    SmartDashboard.putNumber("Feeder Goal", setDistance);
    SmartDashboard.putNumber("Est RPM", estRPM);

  }
}
