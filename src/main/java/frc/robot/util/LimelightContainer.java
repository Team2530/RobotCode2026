package frc.robot.util;

import java.util.ArrayList;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.Odometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight;

import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;

/**
 * The LimelightContainer class manages multiple Limelight cameras and their odometry estimation.
 * It is created once inside RobotContainer, use it with import frc.robot.RobotContainer.LLContainer;
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
      LimelightHelpers.SetFiducialIDFiltersOverride(limelight.getID(), validTagIDs); // makes sure the helper only considers the specified valid tag IDs.
      LimelightHelpers.SetIMUMode(limelight.getID(), 4);
      limelight.setEnabled(true);
    }
  }

  /** Enable or disable all limelights. */
  public void enableLimelights(boolean enable) {
    for (Limelight limelight : limelights) {
      limelight.setEnabled(enable);
    }
  }

  public static void estimateSimOdometry() {
    for (Limelight limelight : limelights) {
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getID());
      if (mt2 != null && mt2.tagCount > 0) {
        SmartDashboard.putString(limelight.getID() + " Pose: ", mt2.pose.toString() + SIMCOUNTER);
        SIMCOUNTER++;
      }
    }
  }
  /** 
   * For every limelight in the container, addVisionMesurement and get robot orientation from pigeon.
   */
  public void estimateMT2Odometry(SwerveDrive swerveDrive) {
    for (Limelight limelight : limelights) {
      boolean doAddVision = true;
      LimelightHelpers.SetRobotOrientation(limelight.getID(), swerveDrive.getYaw().getDegrees()-35, 0, 0, 0, 0, 0);
      
      LimelightHelpers.PoseEstimate mt2Estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getID());

      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      if (
        mt2Estimation != null
        && mt2Estimation.tagCount > 0
      ){
        doAddVision = false;
      } else {
        swerveDrive.addVisionMeasurement(mt2Estimation.pose, mt2Estimation.timestampSeconds);
      }
    }
  }

  public void snapToVision(SwerveDrive swerve) {
    for (Limelight limelight : limelights) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getID());
      if (mt1 != null && mt1.tagCount > 0 ) {
        swerve.resetOdometry(mt1.pose);
        return;
      }
    }
  }

  // public void estimateMT1OdometryPrelim(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, Pigeon2 pigeon,
  //     SwerveModulePosition[] swerveModulePositions) {
  //   for (Limelight limelight : limelights) {
  //     LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

  //     if (mt1 == null || mt1.tagCount == 0 || doRotationRejection(pigeon, 720)) {
  //       continue;
  //     }
      
  //     odometry.resetPosition(mt1.pose.getRotation(), swerveModulePositions, mt1.pose);
  //     SmartDashboard.putString("Pos MT1 prelim: ", mt1.pose.toString() + " " + RLCountermt1);
  //     limelight.pushPoseToNT(mt1.pose);
  //     RLCountermt1++;
  //   }
  // }

  /** Returns if the pigeon detects high angular velocity in degrees per second */
  private boolean doRotationRejection(Pigeon2 pigeon, int dps) {
    return Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) > dps;
  }
  private boolean doRotationRejection(SwerveIMU gyro, double maxDps) {
    // TODO: Check if this is correct
    return Math.abs(Units.radiansToDegrees(gyro.getYawAngularVelocity().magnitude())) > maxDps;
  }
}
