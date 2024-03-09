
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase {
  

  private CANSparkMax elevatorMotor1;
  private CANSparkMax elevatorMotor2;
  //private SparkPIDController pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final PIDController pid;
  private double minPowerAtExtended = 0.00;

  private double newTargetPosition;

  public Elevator() 
  {
    elevatorMotor1 = new CANSparkMax(Constants.Elevator.ElevatorMotor1_ID, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(Constants.Elevator.ElevatorMotor2_ID, MotorType.kBrushless);
    elevatorMotor1.restoreFactoryDefaults();
    elevatorMotor2.restoreFactoryDefaults();
    elevatorMotor1.follow(elevatorMotor2,true);
    

    elevatorMotor1.setIdleMode(IdleMode.kBrake);
    elevatorMotor2.setIdleMode(IdleMode.kBrake);
    elevatorMotor1.setSmartCurrentLimit(20);
    elevatorMotor2.setSmartCurrentLimit(20); 

    elevatorMotor2.getEncoder().setPositionConversionFactor(1);
    
  //  pidController = elevatorMotor2.getPIDController();
    m_encoder = elevatorMotor2.getEncoder();
  //  pidController.setFeedbackDevice(m_encoder);
    newTargetPosition = elevatorMotor2.getEncoder().getPosition();

    //PID
    kP = 0.1; //how aggresive towards target
    kI = 0; //accumlation of past errors
    kD = 0.0001; //how rate of change responds
    kMaxOutput = 0.3; //max motor speed
    kMinOutput = 0.3; //max motor speed in oppisite direction 
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(0);
    //SmartDashboard.putBoolean("Display Elevator", displaySmartDashboard);
  }

  public void GoToHeight(double height)
  {
    newTargetPosition = height;
  }

  
  @Override
  public void periodic() 
  {
    // read PID coefficients from SmartDashboard
    elevatorMotor2.set(getPidOutput());
   /*  displaySmartDashboard = SmartDashboard.getBoolean("Display Elevator", displaySmartDashboard);
    if(displaySmartDashboard)
    {
      SmartDashboard.putNumber("SetPoint", newTargetPosition);
      SmartDashboard.putNumber("Elevator Encoder", elevatorMotor1.getEncoder().getPosition());
    }*/
  }
  public double getPidOutput() {
    double speed = pid.calculate(getDegrees(), newTargetPosition) + getFeedForward();
    if (speed >= 1) {
      return 1;
    }
    else if (speed <= -1) {
      return -1;
    }
    return speed;
  }
  public boolean atSetpoint() {
    if((getDegrees() < newTargetPosition + 3) && (getDegrees() > newTargetPosition - 3))
      return true;
    else
      return false;
  }
  public double getFeedForward() {
    return Math.cos(Math.toRadians(getDegrees()))*getMinPower();
  }

  public double getMinPower() {
    return minPowerAtExtended;
  }
  public double getDegrees() {
    return m_encoder.getPosition();
  }
  public void setmaxcurrent()
  {
    elevatorMotor1.setSmartCurrentLimit(80);
    elevatorMotor2.setSmartCurrentLimit(80); 
  }
}
