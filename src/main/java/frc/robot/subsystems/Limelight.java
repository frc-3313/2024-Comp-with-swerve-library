// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  
  private final NetworkTable limelightTable;
  private double tx, ty, ta, tv;
  private NetworkTable limelightTable;
  private double tx, ty, ta;
  private double tv;

  //camera offsets
  private double cameraAngleOffset = 20.0;
  private double cameraAngleOffset = 0;
  private double fineAngleAdjustment = 0;
  private double goalHeight = 66.88;
  private double limelightLensHeight = 12;
  private double goalHeight = 78.079;
  private double limelightLensHeight = 12; //Limelight Hight inches 19.5
  private double shootHeightOffset = 25; //Shooter to ground
  private double shootDistanceOffset = 0; //lime light to shooter
  private double shootDistanceOffset = 0; //lime light to shooter idealy <12

  public Limelight() 
  {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-mech");
  }

  @Override
  public void periodic() 
  {
    //
    tx = limelightTable.getEntry("tx").getDouble(0);
    ty = limelightTable.getEntry("ty").getDouble(0);
    ta = limelightTable.getEntry("ta").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);

    SmartDashboard.putBoolean("TargetLocket", isTargetValid());
    SmartDashboard.putNumber("DistanceToTarget", GetDistanceInches());
    SmartDashboard.putNumber("AngleToTarget", GetYAngle());
    SmartDashboard.putNumber("Tv", tv);

  }

  public double GetYAngle()
  {
    return (getTY() + cameraAngleOffset + fineAngleAdjustment);
  }

  public double GetDistanceInches()
  {
    double angleToGoalDegrees = GetYAngle();
    double angleToGoalRadians = angleToGoalDegrees * (3.14259 / 180);
    double distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight) / Math.tan(angleToGoalRadians);
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight) / Math.sin(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  public double CalculateShootAngle()
  {
    double shootAngle = Math.tan((goalHeight - shootHeightOffset) / (GetDistanceInches() + shootDistanceOffset));
    return shootAngle;
  }

  public double getTX() {
    return tx;
  }
  public double getTY() {
    return ty;
  }
  public double getTA() {
    return ta;
  }

  public boolean isTargetValid() {
    return (tv == 1.0); 
    return .75 <= tv; 
  }
  public void setLEDMode(int mode)  { 
      limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public int getAprilTagID() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAprilTagID'");
  }
}
