package frc.robot.subsystems.limelight;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.limelight.Reading.ReadingType;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{

    private final String id;
    private final Supplier<Pose3d> cameraPoseSupplier;

    private final StructPublisher<Pose3d> cameraPosePublisher;
    private final StructPublisher<Pose2d> estimatePosePublisher;

    public Limelight(
        String id,
        Supplier<Pose3d> cameraPoseSupplier
    ) {
        this.id = id;
        this.cameraPoseSupplier = cameraPoseSupplier;

        this.cameraPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic(
                    "SmartDashboard/Odometry/" + getId() + "/camera_pose",
                    Pose3d.struct
            ).publish();
        this.estimatePosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic(
                    "SmartDashboard/Odometry/" + getId() + "/estimate_pose",
                    Pose2d.struct
            ).publish();


        LimelightHelpers.SetFiducialIDFiltersOverride(
                getId(),
                LimelightConstants.VALID_TAGS
            );
    }

    public Limelight(
        String id,
        Pose3d cameraPose
    ) {
        this(
            id,
            () -> cameraPose
        );
    }

    @Override
    public void periodic() {
        updatePosition();
    }

    public void updatePosition() {
        Pose3d currentPose = getPose();
        cameraPosePublisher.set(currentPose);

        LimelightHelpers.setCameraPose_RobotSpace(
            id,
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getZ(),
            Units.radiansToDegrees(
                currentPose.getRotation().getX()
            ),
            Units.radiansToDegrees(
                currentPose.getRotation().getY()
            ),
            Units.radiansToDegrees(
                currentPose.getRotation().getZ()
            )
        );

        LimelightHelpers.SetRobotOrientation(
            getId(),
            //swerve.getRotation().getDegrees(),
            RobotContainer.swerveDriveSubsystem.getHeading().getDegrees(),
            0,
            0,
            0,
            0,
            0
        );
    }

    public Optional<Reading> getMT1Reading() {
        updatePosition();

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(
                getId()
            );

        double[] alldevs = NetworkTableInstance.getDefault()
            .getTable(getId())
            .getEntry("stddevs")
            .getDoubleArray(new double[12]);

        int imuMode = NetworkTableInstance.getDefault()
            .getTable(getId())
            .getEntry("imumode_set")
            .getNumber(0)
            .intValue();

        if (
            estimate != null
            && alldevs.length >= 12
        ) {
            double[] stddevs = {
                    alldevs[0],
                    alldevs[1],
                    alldevs[5]
                };
            Reading reading = new Reading(
                    ReadingType.MT1,
                    estimate,
                    stddevs,
                    imuMode,
                    getId()
                );
            
            publish(reading);

            return Optional.of(
                    reading
                );
        } else {
            return Optional.empty();
        }

    }
    
    public Optional<Reading> getMT2Reading() {
        updatePosition();

        PoseEstimate estimate = LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(
                getId()
            );

        double[] alldevs = NetworkTableInstance.getDefault()
            .getTable(getId())
            .getEntry("stddevs")
            .getDoubleArray(new double[12]);

        int imuMode = NetworkTableInstance.getDefault()
            .getTable(getId())
            .getEntry("imumode_set")
            .getNumber(0)
            .intValue();

        if (
            estimate != null
            && alldevs.length >= 12
        ) {
            double[] stddevs = {
                    alldevs[6],
                    alldevs[7],
                    alldevs[11]
                };
            Reading reading = new Reading(
                    ReadingType.MT2,
                    estimate,
                    stddevs,
                    imuMode,
                    getId()
                );

            publish(reading);

            return Optional.of(
                    reading
                );
        } else {
            return Optional.empty();
        }
    }

    public void publish(Reading reading) {
        SmartDashboard.putString(
            "SmartDashboard/Odometry/" + getId() + "/type",
            reading.getType().toString()
        );

        estimatePosePublisher.set(reading.getEstimate().pose);

        SmartDashboard.putNumberArray(
            "SmartDashboard/Odometry/" + getId() + "/stddevs",
            reading.getStddevs()
        );

        SmartDashboard.putNumber(
            "SmartDashboard/Odometry/" + getId() + "/imuMode",
            reading.getIMUMode()
        );
    }

    /*
     * returns the id of the limelight, like "limelight-fr"
     * @return String id
     */
    public String getId() {
        return id;
    }

    /*
     * get the pose of the limelight. this is mean to be a function, in case
     * of scenarios where a given limelight is moving relative to the robot,
     * such as when mounted on a manipulator.
     * @return Pose3d pose - the current pose of the limelight, robot relative
     */
    public Pose3d getPose() {
        return cameraPoseSupplier.get();
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(
                getId(),
                mode
            );
    }

    public void setAlphaAssist(double assist) {
        LimelightHelpers.SetIMUAssistAlpha(
                getId(),
                assist
            );
    }

    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(
            getId(),
            index
        );
    }
}
