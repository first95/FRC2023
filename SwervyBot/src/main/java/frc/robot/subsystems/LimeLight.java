// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;

public class LimeLight extends SubsystemBase {
    private final NetworkTable limelight_data;
    private String _hostname;
    private boolean _aprilTagDetetor;

    // LimeLight data //
    private double[] botpose;

    public LimeLight(String hostname, boolean aprilTagDetetor) {
        _hostname = hostname;
        _aprilTagDetetor = aprilTagDetetor;
        limelight_data = NetworkTableInstance.getDefault().getTable("limelight-" + hostname);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getTlData() {
        return limelight_data.getEntry("tl").getDouble(0);
    }

    public Pose3d getBotPose() {
        if (limelight_data.getEntry("tv").getDouble(0) == 0) {
            return null;
        }

        double[] poseComponents = limelight_data.getEntry("botpose").getDoubleArray(new double[6]);
        Pose3d camPose = new Pose3d(
                poseComponents[0],
                poseComponents[1],
                poseComponents[2],
                new Rotation3d(
                        poseComponents[3],
                        poseComponents[4],
                        poseComponents[5]));

        Pose3d robotPose;
        Alliance alliance = DriverStation.getAlliance();
        
        if (alliance == Alliance.Blue) {
            robotPose = camPose.plus(new Transform3d(Constants.FIELD_CENTER, new Rotation3d()));
        } else if (alliance == Alliance.Red) {
            robotPose = new Pose3d(
                    camPose.getTranslation().unaryMinus().plus(Constants.FIELD_CENTER),
                    camPose.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI)));
        } else {
            return null;
        }

        return robotPose;
    }
}