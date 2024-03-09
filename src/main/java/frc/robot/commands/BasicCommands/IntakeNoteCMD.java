// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNoteCMD extends Command 
{
  public Intake intake;
  public Shooter shooter;

  public IntakeNoteCMD(Intake m_intake, Shooter m_Shooter)
  {
    addRequirements(intake, shooter);
    intake = m_intake;
    shooter = m_Shooter;
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(!intake.hasNote() && !shooter.hasNote())
    {
      intake.DeployIntake();
      intake.RunIntake(.32);
    }
    else
    {
      intake.RetractIntake();
      intake.StopIntake();
    }
    //SmartDashboard.putBoolean("intake is done", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      intake.StopIntake();
      intake.RetractIntake();
      SmartDashboard.putBoolean("intake is done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.hasNote() && intake.atSetpoint() || shooter.hasNote())
    {
      intake.StopIntake();
      intake.RetractIntake();
      return true;
    }
    else
    {
      return false;
    }
  }
}
