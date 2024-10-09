// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Tilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
public class Autotarget extends Command {

  private final Limelight limelight;
  private final Tilter tilter;
  private final SwerveSubsystem driveSubsystem;
  private final CommandXboxController driveController;
  private final PIDController steeringPID;
  private double targetDistance;
  private double kP = 0.18; // Proportional gain
  private final double kI = 0.0; // Integral gain
  private  double kD = 0.005; // Derivative gain
  boolean firsttime = true;
  private double angle = 30; // angle of the goal from the shooter
  private int speakerID;

  /** Creates a new Autotarget. */
  public Autotarget(Limelight limelight, SwerveSubsystem drive,Tilter tilter, CommandXboxController controller) {
    this.limelight = limelight;
    this.driveSubsystem = drive;
    this.tilter = tilter;
    this.driveController = controller;
    this.steeringPID = new PIDController(kP, kI, kD);
    addRequirements(limelight, drive, tilter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(DriverStation.getAlliance().get() == Alliance.Red)
    {
      speakerID = 4;
    }
    else
    {
      speakerID = 8;
    }

    steeringPID.setP(kP);
    steeringPID.setD(kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationX = driveController.getRawAxis (1);
    double translationY = driveController.getRawAxis (0);
    double rotation = driveController.getRightX();

    SmartDashboard.putBoolean("target Valid Auto", limelight.isTargetValid());
    SmartDashboard.putNumber("Distance to April Tag", targetDistance);
    

    if (limelight.isTargetValid()){ 
      int detectedTagID = limelight.getAprilTagID();
      if (detectedTagID == speakerID) {
        double tx = limelight.getTX();
        targetDistance = limelight.GetDistanceInches();
        // Use PID to calculate steering adjustment
        double steeringAdjust = steeringPID.calculate(tx, 0.0); // Aim for zero offset

        // Determine speed based on distance to target (implement your distance logic here)
        double speed = calculateSpeedToTarget();
            
        // Convert steering adjustment to swerve drive inputs
        double autoRotation = MathUtil.clamp(steeringAdjust, -1.0, 1.0); // Ensure rotation speed is within limits
            
        // Drive the swerve robot
        driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(translationX) * driveSubsystem.GetMaxVelocity(),
                            driveSubsystem.powerof2(translationY) * driveSubsystem.GetMaxVelocity()),
                            driveSubsystem.powerof2(autoRotation) * driveSubsystem.GetMaxAngularVelocity(),
                        true);

        // Tells the tilter to go to angle to get to the goal
        angle = limelight.CalculateShootAngle();
        SmartDashboard.putNumber("AngleToGoal", angle);
        tilter.GoToPosition(angle);

      } else {
        // Stop if target is not correct
        // Done maxVelocity and maxAngularVelocity
        driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(translationX) * driveSubsystem.GetMaxVelocity(),
                            driveSubsystem.powerof2(translationY) * driveSubsystem.GetMaxVelocity()),
                            driveSubsystem.powerof2(rotation) * driveSubsystem.GetMaxAngularVelocity(),
                        true);
      }
    } else {
          // Stop if no target is visible
          driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(translationX) * driveSubsystem.GetMaxVelocity(),
                            driveSubsystem.powerof2(translationY) * driveSubsystem.GetMaxVelocity()),
                            driveSubsystem.powerof2(rotation) * driveSubsystem.GetMaxAngularVelocity(),
                        true);
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
