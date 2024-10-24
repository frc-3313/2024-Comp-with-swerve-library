// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ObjectaimCommand extends Command{
  /** Creates a new aimCommand. */
  
  // private final NetworkTable limelightTable;
  private final PIDController steeringPID;
  private final SwerveSubsystem driveSubsystem; // Assume you have a drive subsystem
  private final double kP = 0.1; // Proportional gain
  private final double kI = 0.0; // Integral gain
  private final double kD = 0.0; // Derivative gain
    
  public ObjectaimCommand(SwerveSubsystem drive)
  {
    this.driveSubsystem = drive;
    this.steeringPID = new PIDController(kP, kI, kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
     // Get the data from the Limelight
    double tx = LimelightHelpers.getTX(Constants.Limelight.FRONT); // Horizontal offset
    boolean targetVisible = LimelightHelpers.getTV(Constants.Limelight.FRONT);// Target visibility
    Translation2d translation = new Translation2d();  // Adjust as necessary
    if (targetVisible) 
    {
      // Use PID to calculate steering adjustment
      double steeringAdjust = steeringPID.calculate(tx, 0.0); // Aim for zero offset

      // Convert steering adjustment to swerve drive inputs           
      double rotation = MathUtil.clamp(steeringAdjust, -1.0, 1.0); // Ensure rotation speed is within limits
      
      // Drive the swerve robot
      driveSubsystem.drive(translation, rotation, false);
    } 
    else 
    {
      // Stop if no target is visible
      driveSubsystem.drive(translation, 0, false);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
