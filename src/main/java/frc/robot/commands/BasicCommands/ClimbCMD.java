// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;

public class ClimbCMD extends Command 
{

  Elevator elevator;
  Tilter tilter;

  public ClimbCMD(Elevator elevator, Tilter tilter) {
    this.elevator = elevator;
    this.tilter = tilter;
    addRequirements(elevator, tilter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    elevator.GoToHeight(Constants.Elevator.elvAmpPosition);
    tilter.GoToPosition(Constants.Tilter.ampPosition);
    elevator.setMotorAmp(80);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    elevator.GoToHeight(Constants.Elevator.climbLowPosition);
    elevator.setMotorBrake();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
