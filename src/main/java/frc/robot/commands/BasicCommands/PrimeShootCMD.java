// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class PrimeShootCMD extends InstantCommand {

  public Tilter tilter;
  public Shooter shooter;
  public Elevator elevator;
  private Double shootAngle;
  private Double elevatorHeight;
  private double shootSpeed;
  public Timer timer;
  private boolean noteWasToClose;

  
  public PrimeShootCMD(Tilter tilter, Shooter shooter, Elevator elevator, Double shootSpeed, Double shootAngle, Double elevatorHeight) 
  {
    this.tilter = tilter;
    this.shooter = shooter;
    this.elevator = elevator;
    this.shootAngle = shootAngle;
    this.elevatorHeight = elevatorHeight;
    this.shootSpeed = shootSpeed;
    addRequirements(tilter, shooter, elevator);
  }

  @Override
  public void initialize() 
  {
    timer = new Timer();
    noteWasToClose = false;
    timer.start();
    if(!shooter.noteToClose())
    { 
      shooter.SetShooterSpeed(shootSpeed);
      elevator.GoToHeight(elevatorHeight);
      tilter.GoToPosition(shootAngle);
    }
    else
    {
      noteWasToClose = true;
     shooter.MoveFeederDistance(Constants.Shooter.FeederBackDistance);
    }
    
  }
  @Override
  public void execute() 
  {
    if(!shooter.noteToClose() || timer.hasElapsed(.2))
    { 
      shooter.SetShooterSpeed(shootSpeed);
      elevator.GoToHeight(elevatorHeight);
      tilter.GoToPosition(shootAngle);
    }
  }
    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((shooter.hasNote() && timer.hasElapsed(.2)) || !noteWasToClose)
    {
      return true;
    }
    else
    {
      return false;
    }
    
  }

}
