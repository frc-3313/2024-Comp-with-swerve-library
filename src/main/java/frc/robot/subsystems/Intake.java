
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase 
{

  //Intake Motor Setup
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.IntakeMotor_ID, MotorType.kBrushless);

  public Intake() 
  {
     
    intakeMotor.setIdleMode(IdleMode.kBrake);

  }

  public void RunIntake(double speed)
  {
    intakeMotor.set(speed);
   
  }
  public void StopIntake()
  {
    intakeMotor.set(0);
  }


  @Override
  public void periodic() 
  {
    
    if(intakeMotor.getEncoder().getVelocity() > 10)
    {
      SmartDashboard.putBoolean("Intake running", true);
    }
    else
    {
      SmartDashboard.putBoolean("Intake running", false);
    }
  }
  
}
