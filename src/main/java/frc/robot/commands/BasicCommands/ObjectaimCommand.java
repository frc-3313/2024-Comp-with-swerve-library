// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class aimCommand extends Command{
  /** Creates a new aimCommand. */
  
    // private final NetworkTable limelightTable;
    private final PIDController steeringPID;
    private final SwerveSubsystem driveSubsystem; // Assume you have a drive subsystem
    private final Limelight limelight;
    private final double kP = 0.1; // Proportional gain
    private final double kI = 0.0; // Integral gain
    private final double kD = 0.0; // Derivative gain
    
    public aimCommand(SwerveSubsystem drive, Limelight limelight)
    {

    
        this.driveSubsystem = drive;
        this.limelight = limelight;
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
        double tx = limelight.getTX(); // Horizontal offset
        boolean targetVisible = limelight.isTargetValid();// Target visibility
        Translation2d translation = new Translation2d();  // Adjust as necessary
        if (targetVisible) {
            // Use PID to calculate steering adjustment
            double steeringAdjust = steeringPID.calculate(tx, 0.0); // Aim for zero offset

            // Determine speed based on distance to target (implement your distance logic here)
            double speed = calculateSpeedToTarget();
            
            // Convert steering adjustment to swerve drive inputs
           
            double rotation = MathUtil.clamp(steeringAdjust, -1.0, 1.0); // Ensure rotation speed is within limits
            
            // Drive the swerve robot
            driveSubsystem.drive(translation, rotation, false);
        } else {
            // Stop if no target is visible
            driveSubsystem.drive(translation, 0, false);
        }
    }
    private double calculateSpeedToTarget() {
      // Implement logic to calculate the speed based on your desired approach distance
      // For example, you could adjust speed based on distance from the target
      return 0.5; // Replace with actual speed calculation based on your logic
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
