// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.hardware.Pigeon2;

//will most likely need to import other things as we go along, some are currently unused, keep them for now.

/**
 * The LimelightContainer class manages multiple Limelight cameras and their
 * odometry estimation.
 */
public class LimelightContainer {
  static int SIMCOUNTER = 0;
  static int RLCOUNTER = 0;
  static int RLCountermt1 = 0;
  private static ArrayList<Limelight> limelights = new ArrayList<Limelight>();

  public LimelightContainer(Limelight... limelights) {
    // This is were we add valid tag ids.
    int[] validTagIDs = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 }; 
    // it is very likely that we will not use all of these, but just in case we do, they are here. Remove as needed. 
    // Each tag id should be commented here to show what object it is in corralation to the field.

    for (Limelight limelight : limelights) {
      LimelightContainer.limelights.add(limelight);
      LimelightHelpers.SetFiducialIDFiltersOverride(limelight.getName(), validTagIDs); //makes sure the helper only considers the specified valid tag IDs.
      LimelightHelpers.SetIMUMode(limelight.getName(), 0);

      limelight.setEnabled(true);
    }

  }

  public void enableLimelights(boolean enable) {
    for (Limelight limelight : limelights) {
      limelight.setEnabled(enable);
    }
  }
  


  public static void estimateSimOdometry() {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
      
      if (mt2 == null) { // in case not all limelights are connected
        continue;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        limelight.pushPoseToShuffleboard(limelight.getName() + " MT2Pose", mt2.pose);
        SmartDashboard.putString(limelight.getName() + " Pose: ", mt2.pose.toString() + SIMCOUNTER);
        SIMCOUNTER++;
      }
    }
  }
  /** 
   * For every limelight in the container, estimate odometry using MegaTag2 & Push to Shuffleboard
   */
  public void estimateMT2Odometry(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, Pigeon2 pigeon2) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(limelight.getName(), pigeon2.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2Estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());

      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      if (doRotationRejection(pigeon2, 720)) {
        doRejectUpdate = true;
      }
      if (mt2Estimation.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        limelight.pushPoseToShuffleboard(limelight.getName() + " MT2Pose", mt2Estimation.pose);
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(3, 3, 9999999));
        odometry.addVisionMeasurement(mt2Estimation.pose, mt2Estimation.timestampSeconds);
      }
    }
  }

  public void estimateMT1OdometryPrelim(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, Pigeon2 pigeon,
      SwerveModulePosition[] swerveModulePositions) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;

      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

      if (mt1 == null) {
        continue;
      }

      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (doRotationRejection(pigeon, 720)) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        odometry.resetPosition(mt1.pose.getRotation(), swerveModulePositions, mt1.pose);
        SmartDashboard.putString("Pos MT1 prelim: ", mt1.pose.toString() + " " + RLCountermt1);
        limelight.pushPoseToShuffleboard(limelight.getName() + " mt1", mt1.pose);
      }

      RLCountermt1++;
    }
  }

  public void estimateMT1Odometry(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, Pigeon2 pigeon) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

      if (mt1 == null) {
        continue;
      }

      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (mt1.avgTagDist < Units.feetToMeters(2)) // origanally 10ft, for testing set to 2ft.
        doRejectUpdate = true;

      if (doRotationRejection(pigeon,720)) {
        doRejectUpdate = true;
      }

      if ((Math.abs(mt1.pose.getX() - odometry.getEstimatedPosition().getX()) > 1.5)
          || (Math.abs(mt1.pose.getY() - odometry.getEstimatedPosition().getY()) > 1.5)) {

        doRejectUpdate = true;

      }

      if (!doRejectUpdate) {
        // Use realistic vision measurement standard deviations (meters, meters, radians)
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(5.0)));
        odometry.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);

        SmartDashboard.putString("Pos MT1: ", mt1.pose.toString() + " " + RLCountermt1);
        limelight.pushPoseToShuffleboard(limelight.getName() + " mt1", mt1.pose);

      }

      RLCountermt1++;
    }
  }

  /** Returns if the pigeon detects high angular velocity in degrees per second */
  private boolean doRotationRejection(Pigeon2 pigeon, int dps) {
    return Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) > dps;
  }
}