package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import java.util.Arrays;
import java.util.ArrayList;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;


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
    }
  }

  /** Enable or disable all limelights. */
  public void enableLimelights(boolean enable) {
    for (Limelight limelight : limelights) {
      limelight.setEnabled(enable);
    }
  }

  public void setIMUMode(int mode) {
    for (Limelight limelight : limelights) {
      limelight.setIMUMode(mode);
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
  public void estimateMT2Odometry(SwerveSubsystem swerve) {
    for (Limelight limelight : limelights) {
      LimelightHelpers.SetRobotOrientation(
        limelight.getID(), 
        //swerve.getRotation().getDegrees(),
        swerve.getHeading().getDegrees(),
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

      if (mt2Estimation != null) {
        limelight.publish(mt2Estimation.pose);
        /*
        SmartDashboard.putNumberArray(
          "Odometry/mt2/" + limelight.getID() + "/stddevs", 
          Arrays.copyOfRange(
              NetworkTableInstance.getDefault()
                .getTable(limelight.getID())
                .getEntry("stddevs")
                .getDoubleArray(new double[6]), 
              7, 
              11
            )
        ); */
      }
            
  

      boolean added;
      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      if (
        limelight.isEnabled()
        && mt2Estimation != null
        && mt2Estimation.tagCount >= 2
        && swerve.getAngularVelocity().lt(RadiansPerSecond.of(Math.PI / 2))
      ){
        double[] stddevs = NetworkTableInstance.getDefault()
          .getTable(limelight.getID())
          .getEntry("stddevs")
          .getDoubleArray(new double[6]);
        
        if (stddevs.length < 12) {
          continue;
        }

        int imuMode = NetworkTableInstance.getDefault().getTable(limelight.getID()).getEntry("imumode_set").getNumber(0).intValue();
            /* VecBuilder.fill(
              0.7,
              0.7,
              999999
            ) */
        swerve.addVisionMeasurement(
            mt2Estimation.pose,
            mt2Estimation.timestampSeconds,
            VecBuilder.fill(
              stddevs[6],
              stddevs[7],
              imuMode != 1
                ? stddevs[11]
                : 999999
            )
        );
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
  
  public void estimateMT1Odometry(SwerveSubsystem swerve) {
    for (Limelight limelight : limelights) {
      LimelightHelpers.PoseEstimate mt1Estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getID());

      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      
      SmartDashboard.putNumber(
        "Odometry/mt1/" + limelight.getID() + "/lastConsidered",
        Timer.getTimestamp()
      );

      boolean added;
      if (mt1Estimation != null) {
        double[] stddevs = NetworkTableInstance.getDefault().getTable(limelight.getID()).getEntry("stddevs").getDoubleArray(new double[6]);

        if (stddevs.length < 12) {
          continue;
        }
        SmartDashboard.putNumberArray(
          "Odometry/mt1/" + limelight.getID() + "/stddevs", Arrays.copyOfRange(
            stddevs, 0, 5
            )
        );

      if (
        limelight.isEnabled()
        && mt1Estimation.tagCount > 0
        && swerve.getAngularVelocity().lt(RadiansPerSecond.of(Math.PI * 2))
      ){

        swerve.addVisionMeasurement(
            mt1Estimation.pose,
            mt1Estimation.timestampSeconds,
            VecBuilder.fill(
              stddevs[0],
              stddevs[1],
              stddevs[5]
            )
        );
        added = true;
      } else {
        added = false;
      } 
      } else {
        added = false;
      }
      SmartDashboard.putBoolean(
        "Odometry/mt1/" + limelight.getID() + "/added",
        added
      );
    }
  }

  public void snapToVision(SwerveSubsystem swerve) {
    for (Limelight limelight : limelights) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getID());
      if (mt1 != null && mt1.tagCount > 0 ) {
        setIMUMode(1);
        swerve.resetOdometry(mt1.pose);
        SetRobotOrientation(
         
        Degrees.of(swerve.getHeading().getDegrees())
        );

        break;
      }
    
    }
    estimateMT1Odometry(swerve);
    setIMUMode(1);
    for (Limelight limelight : limelights) {
      LimelightHelpers.SetRobotOrientation(
        limelight.getID(), 
        swerve.getHeading().getDegrees(),
        0, 
        0, 
        0, 
        0, 
        0
      );
    }
  }

  public void SetRobotOrientation(Angle yaw) {
    for (Limelight limelight: limelights) {
      LimelightHelpers.SetRobotOrientation(
        limelight.getID(), 
        //swerve.getRotation().getDegrees(),
        yaw.in(Degrees),
        0, 
        0, 
        0, 
        0, 
        0
      );
    }
  }
}
