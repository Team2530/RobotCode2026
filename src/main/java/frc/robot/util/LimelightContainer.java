package frc.robot.util;

import java.util.ArrayList;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.Odometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
      LimelightHelpers.SetIMUAssistAlpha("limelight", 0.001);
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
      boolean doRejectUpdate = false;
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
      if (mt2 == null) { // in case not all limelights are connected
        continue;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        SmartDashboard.putString(limelight.getName() + " Pose: ", mt2.pose.toString() + SIMCOUNTER);
        SIMCOUNTER++;
      }
    }
  }
  /** 
   * For every limelight in the container, addVisionMesurement and get robot orientation from pigeon.
   */
  public void estimateMT2Odometry(SwerveDrive swerveDrive) {
    for (Limelight limelight : limelights) {
      SwerveIMU gyro = swerveDrive.getGyro();
      boolean doAddVision = true;
      LimelightHelpers.SetRobotOrientation(
        limelight.getID(), 
        Units.radiansToDegrees(
          (
            (Math.PI / 2)
            + swerveDrive.getGyroRotation3d().getZ()
          ) % (Math.PI * 2)
        ),
        0, 
        0, 
        0, 
        0, 
        0
      );
      
      LimelightHelpers.PoseEstimate mt2Estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getID());

      SmartDashboard.putNumber(
        "Odometry/mt2/" + limelight.getID() + "/lastConsidered",
        Timer.getTimestamp()
      );

      boolean added;
      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      if (
        limelight.isEnabled()
        && mt2Estimation != null
        && mt2Estimation.tagCount >= 3
        && doRotationRejection(gyro, 360)
      ){
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 99999));
        swerveDrive.addVisionMeasurement(mt2Estimation.pose, mt2Estimation.timestampSeconds);
        // add set vision measurments and add vision measurments here instead of the else statment. 
        added = true;
      } else {
        added = false;
      }

      SmartDashboard.putBoolean(
        "Odometry/mt2/" + limelight.getID() + "/added",
        added
      );
    }
  }
  
  public void estimateMT1Odometry(SwerveDrive swerveDrive) {
    for (Limelight limelight : limelights) {
      SwerveIMU gyro = swerveDrive.getGyro();
      boolean doAddVision = true;
      LimelightHelpers.SetRobotOrientation(
        limelight.getID(), 
        Units.radiansToDegrees(
          (
            (Math.PI / 2)
            + swerveDrive.getGyroRotation3d().getZ()
          ) % (Math.PI * 2)
        ),
        0, 
        0, 
        0, 
        0, 
        0
      );
      
      LimelightHelpers.PoseEstimate mt1Estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getID());

      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      
      SmartDashboard.putNumber(
        "Odometry/mt1/" + limelight.getID() + "/lastConsidered",
        Timer.getTimestamp()
      );

      boolean added;
      if (
        limelight.isEnabled()
        && mt1Estimation != null
        && mt1Estimation.tagCount > 0
        && doRotationRejection(gyro, 720)
      ){
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 99999));
        swerveDrive.addVisionMeasurement(mt1Estimation.pose, mt1Estimation.timestampSeconds);
        added = true;
      } else {
        added = false;
      }

      SmartDashboard.putBoolean(
        "Odometry/mt2/" + limelight.getID() + "/added",
        added
      );
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

  /*
  public void estimateMT1OdometryPrelim(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, Pigeon2 pigeon,
      SwerveModulePosition[] swerveModulePositions) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;

      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

      if (mt1 == null) {continue;}
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (doRotationRejection(pigeon, 720)) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        odometry.resetPosition(mt1.pose.getRotation(), swerveModulePositions, mt1.pose);
        SmartDashboard.putString("Pos MT1 prelim: ", mt1.pose.toString() + " " + RLCountermt1);
      }
      RLCountermt1++;
    }
  } */

  /** Returns if the pigeon detects high angular velocity in degrees per second */
  private boolean doRotationRejection(Pigeon2 pigeon, int dps) {
    return Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) > dps;
  }
  private boolean doRotationRejection(SwerveIMU gyro, double maxDps) {
    // TODO: Check if this is correct
    return Math.abs(Units.radiansToDegrees(gyro.getYawAngularVelocity().magnitude())) > maxDps;
  }
}
