// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  
  private NetworkTable limelightTable;
  private double tx, ty, ta;
  private double tv;
  //Limelight Offsets
  private double cameraAngleOffset = 41; //Should be angle that the Limelight is facing in degrees, but in actuality, it is not. 41 just seems to work.
  private double goalHeight = 78; //Height of the speaker in inches
  private double limelightLensHeight = 8.25; //Limelight height in inches
  private double shootHeightOffset = 10; //Height from Limelight to Shooter pivot in inches
  private double shootDistanceOffset = 9; //Distance from Limelight to Shooter pivot in inches
  private double fineDistanceAdjustment = 12; // Distance away from the wall in inches, to adjust for note drop

  public Limelight() 
  {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-mech");
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("DistanceToTarget", GetDistanceInches());

    tx = limelightTable.getEntry("tx").getDouble(0);
    ty = limelightTable.getEntry("ty").getDouble(0);
    ta = limelightTable.getEntry("ta").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);
    
  }

  public double GetYAngle()
  {
    return (getTY() + cameraAngleOffset);
  }

  public double GetDistanceInches()
  {
    double angleToGoalDegrees = GetYAngle();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  public double CalculateShootAngle(double distance)
  {
    return ((-0.278)*distance)+(66.62);
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
    return .75 <= tv; 
  }
  public void setLEDMode(int mode)  { 
      limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public int getAprilTagID() {
    return (int) limelightTable.getEntry("tid").getDouble(-1);
  }
}