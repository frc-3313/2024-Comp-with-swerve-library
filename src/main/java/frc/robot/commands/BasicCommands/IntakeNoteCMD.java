// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNoteCMD extends Command 
{
  public Intake intake;
  public Shooter shooter;
  public Tilter tilter;
  public Timer timer;
  public boolean timerStarted;

  public IntakeNoteCMD(Intake m_intake, Shooter m_Shooter, Tilter m_Tilter)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = m_intake;
    shooter = m_Shooter;
    tilter = m_Tilter;
    addRequirements(shooter, intake, tilter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer = new Timer();
    timerStarted = false;
    if(!shooter.hasNote())
    {
      tilter.GoToPosition(Constants.Tilter.handOffPosition);
    }
    else
    {
      intake.StopIntake();
      shooter.StopFeeder();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(tilter.atSetpoint())
    {
      shooter.StartFeeder(.2);
      intake.RunIntake(.4);
    }
    if(shooter.hasNote())
    {
      intake.StopIntake();
      shooter.StopFeeder();
      if(!timerStarted)
      {
        timer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      intake.StopIntake();
      
      SmartDashboard.putBoolean("intake is done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.hasNote() && timer.hasElapsed(.2))
    {
      if(shooter.noteToClose())
      {
        shooter.MoveFeederDistance(Constants.Shooter.FeederBackDistance);
      }   
      return true;
    }
    else
    {
      return false;
    }
  }
}
