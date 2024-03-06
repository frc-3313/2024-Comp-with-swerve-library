
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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

public class Intake extends SubsystemBase 
{
  //Tilter Motor Setup
  private final CANSparkMax intakeTilterMotor = new CANSparkMax(Constants.Intake.IntakeTilterMotor_ID, MotorType.kBrushless);
  private static final SparkAbsoluteEncoder.Type kAltEncType = SparkAbsoluteEncoder.Type.kDutyCycle;

  private SparkPIDController pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private AbsoluteEncoder alternateEncoder;

  private double newTargetPosition;

  //Intake Motor Setup
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.IntakeMotor_ID, MotorType.kBrushless);

  //Distance sensor
   private ColorSensorV3 distanceSensor = new ColorSensorV3(Port.kOnboard); 

  private Boolean displaySmartDashboard;

  public Intake() 
  {
    displaySmartDashboard = false;
    intakeTilterMotor.restoreFactoryDefaults();
    intakeTilterMotor.setIdleMode(IdleMode.kBrake);
    alternateEncoder = intakeTilterMotor.getAbsoluteEncoder(kAltEncType);
    alternateEncoder.setZeroOffset(.5);
    alternateEncoder.setPositionConversionFactor(360);
    pidController = intakeTilterMotor.getPIDController();
    pidController.setFeedbackDevice(alternateEncoder);
    newTargetPosition = Constants.Intake.RetractPosition;
    intakeMotor.setIdleMode(IdleMode.kBrake);

    //PID
    kP = .005; //how aggresive towards target
    kI = 0; //accumlation of past errors
    kD = .001; //how rate of change responds
    kIz = 0;  //integral zone how much of the zone it looks at
    kFF = 0;  //feed forward again estimates the future based on past
    kMaxOutput = .5; //max motor speed
    kMinOutput = -.5; //max motor speed in oppisite direction 

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putBoolean("Display Intake", displaySmartDashboard);
  }

  public void DeployIntake()
  {
    newTargetPosition = Constants.Intake.DeployPosition;
  }
  public void RetractIntake()
  {
    newTargetPosition = Constants.Intake.HandOffPosition;
  }

  public void RunIntake(double speed)
  {
    intakeMotor.set(speed);
  }
  public void StopIntake()
  {
    intakeMotor.set(0);
  }
  public boolean hasNote()
  { 
    if(GetDistance() > 58 )
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

   public void GoToPosition(double position)
   {
     newTargetPosition = position;
   }
   public boolean atSetpoint() {
    //return pid.atSetpoint();
    if((getDegrees() < newTargetPosition + 2) && (getDegrees() > newTargetPosition - 2))
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
    displaySmartDashboard = SmartDashboard.getBoolean("Display Elevator", displaySmartDashboard);
    if(displaySmartDashboard)
    {
      pidController.setReference(newTargetPosition, CANSparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("Intake Setpoint", newTargetPosition);
      SmartDashboard.putNumber("intake position", alternateEncoder.getPosition());
      SmartDashboard.putNumber("intake sensor", distanceSensor.getProximity());
      SmartDashboard.putBoolean("intake has note", hasNote());
      SmartDashboard.putBoolean("Intake At Set", atSetpoint());
    }

  }
  
}
