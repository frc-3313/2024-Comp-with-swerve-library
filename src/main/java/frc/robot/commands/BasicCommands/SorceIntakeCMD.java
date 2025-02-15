// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class SorceIntakeCMD extends Command {
  public Intake intake;
  public Elevator elevator;
  public Tilter tilter;
  public Shooter shooter;
  public Timer timer;
  public boolean timerStarted;
  /** Creates a new AmpScoreCMD. */
  public SorceIntakeCMD(Intake m_Intake, Elevator m_Elevator, Tilter m_Tilter, Shooter m_Shooter){
    intake = m_Intake;
    elevator = m_Elevator;
    tilter = m_Tilter;
    shooter = m_Shooter;
    addRequirements(intake, tilter, shooter, elevator);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer = new Timer();
    timerStarted = false;
  // tilter.GoToPosition(Constants.Tilter.ampPosition); 
  // shooter.StartShooter();
    if (!shooter.hasNote()) 
    {
      elevator.GoToHeight(Constants.Elevator.SorceIntakePosition);
      tilter.GoToPosition(Constants.Tilter.shootFromStage);
      shooter.SetShooterSpeed(Constants.Shooter.sourceIntakeSpeed);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(shooter.noteToClose())
    {
      if(!timerStarted)
      {
        timer.start();
        elevator.GoToHeight(Constants.Elevator.elvBottomPosition);   
        tilter.GoToPosition(Constants.Tilter.stowPosition); 
        shooter.StopShooter(); 
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {

    if(shooter.noteToClose() && timer.hasElapsed(.2))
    {
      if(shooter.noteToClose())
      {
        shooter.MoveFeederDistance(Constants.Shooter.FeederBackDistance);
      } 
      if(timer.hasElapsed(.4))
        return true;
      else
        return false;
    }
    else
    {
      if(timer.hasElapsed(.4))
      {  
        return false;
      }
      else
      {
        return true;
      }
    }
  }
}
