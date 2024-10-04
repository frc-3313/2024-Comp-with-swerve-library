// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class Autotarget extends Command {

  private final Limelight limelight;
  private final SwerveSubsystem driveSubsystem;
  private final Shooter shooter;
  private final Tilter tilter;
  private final CommandXboxController driveController;
  private final PIDController steeringPID;
  private final int targetTagID;
  private double targetDistance;
  private double kP = 0.15; // Proportional gain
  private final double kI = 0.0; // Integral gain
  private  double kD = 0.0; // Derivative gain
  boolean firsttime = true;
  /** Creates a new Autotarget. */
  public Autotarget(Limelight limelight, SwerveSubsystem drive, Shooter shooter, Tilter tilter, CommandXboxController controller, int tagID) {
    this.limelight = limelight;
    this.driveSubsystem = drive;
    this.shooter = shooter;
    this.tilter = tilter;
    this.driveController = controller;
    this.targetTagID = tagID;
    this.steeringPID = new PIDController(kP, kI, kD);
    addRequirements(limelight, drive, shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationX = driveController.getRawAxis (1);
    double translationY = driveController.getRawAxis (0);

    SmartDashboard.putBoolean("target Valid Auto", limelight.isTargetValid());
    SmartDashboard.putNumber("Distance to April Tag", targetDistance);
    SmartDashboard.putNumber("targetTagID", targetTagID);
    if(firsttime)
    {
      SmartDashboard.putNumber("kP", kP);
      SmartDashboard.putNumber("kD", kD);  
      firsttime = false; 
    }
    kP = SmartDashboard.getNumber("kP", kP);
    kD = SmartDashboard.getNumber("kD", kD);


    steeringPID.setD(kP);
    steeringPID.setD(kD);

    Translation2d translation = new Translation2d();  // Adjust as necessary
    if (limelight.isTargetValid()){ 
      int detectedTagID = limelight.getAprilTagID();
      if (detectedTagID == targetTagID) {
        double tx = limelight.getTX();
        targetDistance = limelight.GetDistanceInches();
          // Use PID to calculate steering adjustment
          double steeringAdjust = steeringPID.calculate(tx, 0.0); // Aim for zero offset

          // Determine speed based on distance to target (implement your distance logic here)
          double speed = calculateSpeedToTarget();
            
          // Convert steering adjustment to swerve drive inputs
          double rotation = MathUtil.clamp(steeringAdjust, -1.0, 1.0); // Ensure rotation speed is within limits
            
          // Drive the swerve robot
          driveSubsystem.drive(translation, rotation, false);
        }  else {
          // Stop if no target is visible
          driveSubsystem.drive(translation, 0, false);
    }
    } else {
          // Stop if no target is visible
          driveSubsystem.drive(translation, 0, false);
    }
    // if (distanceToTag < targetDistance){
    //   double shootAngle = CalculateShootAngle(distanceToTag);
    //   shooter.setShooterAngle(shootAngle);
    // }
    // else {
    //   //FIX add normal drive code here
    // }

  }
  private double calculateSpeedToTarget() {
    // Implement logic to calculate the speed based on your desired approach distance
    // For example, you could adjust speed based on distance from the target
    return 0.5; // Replace with actual speed calculation based on your logic
  }
  // Called once the command ends or is interrupted.
  private void rotateSwerveToTarget(double tx) {
    if (Math.abs(tx) > 1.0){
      
      //swerveDrive.rotate(rotationalSpeed);
    }
  }

  private double CalculateShootAngle(double distance){
    return 45 - (distance * 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
