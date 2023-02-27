// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSystem extends SubsystemBase {
  /** Creates a new LimelightSystem. */
  private double tv, x, y, area;

  public LimelightSystem() {
    readLimeLight();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    readLimeLight();
  }

  // Reads the values from the Limelight
  public void readLimeLight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    tv = tv.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightTV", tv);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  // Gets the values X, Y, Area, and TV
  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getArea() {
    return area;
  }

  public double getTV() {
    return tv;
  }

  // Sets the LED mode
  public void setLEDMode(int mode) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ledMode = table.getEntry("ledMode");
    ledMode.setNumber(mode);
  }

  // Turns off the LEDS
  public void setLEDOFF() {
    setLEDMode(1);
  }

  // Turns on the LEDS
  public void setLEDON() {
    setLEDMode(3);
  }

}
