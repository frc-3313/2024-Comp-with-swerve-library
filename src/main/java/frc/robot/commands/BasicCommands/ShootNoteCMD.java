// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class ShootNoteCMD extends Command {
  
  public Tilter tilter;
  public Shooter shooter;
  public Elevator elevator;
  public boolean end;

  public ShootNoteCMD(Tilter tilter, Shooter shooter, Elevator elevator) {
    this.tilter = tilter;
    this.shooter = shooter;
    this.elevator = elevator;
    addRequirements(tilter, shooter, elevator);
    end = false;
  }

  @Override
  public void end(boolean interrupted) 
  {
    shooter.StopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(shooter.IsShooterAboveRPM() && tilter.atSetpoint() && elevator.atSetpoint())
    {
      if (!end)
      {
        shooter.MoveFeederDistance(10);
        end = true;
      }
      return false;
    }
    else if(shooter.FeederDone())
    {
      return true;
    }
    else{
      return false;
    }
  }
}
