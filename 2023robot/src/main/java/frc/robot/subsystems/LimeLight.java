// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  private final NetworkTable limelight_data;
  private boolean _isAprilTagDetection;
  private String _hostname;

  // LimeLight data //
  private double[] botpose;

  public LimeLight(String hostname, boolean isAprilTagDetection) {
    _hostname = hostname;
    _isAprilTagDetection = isAprilTagDetection;
    limelight_data = NetworkTableInstance.getDefault().getTable("limelight-" + hostname);
  }

  @Override
  public void periodic() {
    if(_isAprilTagDetection)
      botpose = limelight_data.getEntry("botpose").getDoubleArray(new double[0]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}