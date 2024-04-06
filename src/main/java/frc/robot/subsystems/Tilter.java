
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tilter extends SubsystemBase 
{
  private final CANSparkMax tilterMotor = new CANSparkMax(Constants.Tilter.TilterMotor_ID, MotorType.kBrushless);
  private static final SparkAbsoluteEncoder.Type kAltEncType = SparkAbsoluteEncoder.Type.kDutyCycle;

  private SparkPIDController pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private AbsoluteEncoder alternateEncoder;

  private double newTargetPosition;
  public Tilter() 
  {
    tilterMotor.restoreFactoryDefaults();
    tilterMotor.setIdleMode(IdleMode.kBrake);
    tilterMotor.setInverted(true);
    alternateEncoder = tilterMotor.getAbsoluteEncoder(kAltEncType);
    alternateEncoder.setPositionConversionFactor(360);
    alternateEncoder.setZeroOffset((180));
    pidController = tilterMotor.getPIDController();
    pidController.setFeedbackDevice(alternateEncoder);
    //SmartDashboard.putBoolean("Display Tilter", displaySmartDashboard);

    //PID
    kP = 0.025; //how aggresive towards target
    kI = 0; //accumlation of past errors
    kD = 0.00001; //how rate of change responds
    kIz = 0;  //integral zone how much of the zone it looks at
    kFF = 0;  //feed forward again estimates the future based on past
    kMaxOutput = 1; //max motor speed
    kMinOutput = -1; //max motor speed in oppisite direction 

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    newTargetPosition = Constants.Tilter.shootFromSpeaker;

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  public void GoToPosition(double position)
  {
    newTargetPosition = position;
  }

  public boolean atSetpoint() {
    //return pid.atSetpoint();
    if((getDegrees() < newTargetPosition + 3) && (getDegrees() > newTargetPosition - 3))
      return true;
    else
      return false;
  }

  public boolean atSetpoint(double setpoint) {
    //return pid.atSetpoint();
    if((getDegrees() < newTargetPosition + setpoint) && (getDegrees() > newTargetPosition - setpoint))
      return true;
    else
      return false;
  }

  public double getDegrees() {
    return alternateEncoder.getPosition();
  }
  
  @Override
  public void periodic() 
  {
  
      SmartDashboard.putNumber("tilter absolute enc", alternateEncoder.getPosition());
      SmartDashboard.putBoolean("tilter at set", atSetpoint());
    
    pidController.setReference(newTargetPosition, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("Tilter SetPoint", newTargetPosition);

  }
}
