package frc.robot.subsystems.limelight;

import static edu.wpi.first.units.Units.*;

import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RewindStats;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{

    private final ArrayList<Limelight> limelights;

    public LimelightSubsystem(Limelight... limelights) {
       this.limelights = new ArrayList<>(
               Arrays.asList(limelights)
           );

       setRewinds(true);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setIMUModes(1);
        } else if (
            RobotContainer.swerveDriveSubsystem.getAngularVelocity()
                .abs(DegreesPerSecond)
            > 2
        ) {
            setIMUModes(4);
            setAlphaAssists(0.025);
            
        } else {
            setIMUModes(4);
            setAlphaAssists(0.035);
        }

        Time pipelinePeriod = Milliseconds.of(100);
        if (
            (
                Timer.getTimestamp() 
                % pipelinePeriod.times(2).in(Seconds)
            ) > pipelinePeriod.in(Seconds)
        ) {
            setPipelines(0);
        } else {
            setPipelines(1);
        }
    }

    public ArrayList<Reading> getMT1Readings() {
        ArrayList<Reading> readings = new ArrayList<>();

        for (Limelight limelight: limelights) {
            Optional<Reading> optional = limelight.getMT1Reading();
            
            if (optional.isPresent()) {
                readings.add(optional.get());
            }
        }

        return readings;
    }

    public ArrayList<Reading> getMT2Readings() {
        ArrayList<Reading> readings = new ArrayList<>();

        for (Limelight limelight: limelights) {
            Optional<Reading> optional = limelight.getMT2Reading();
            
            if (optional.isPresent()) {
                readings.add(optional.get());
            }
        }

        return readings;
    }

    public void setIMUModes(int mode) {
        for (Limelight limelight : limelights) {
            LimelightHelpers.SetIMUMode(
                    limelight.getId(), 
                    mode
            );
        }
    }

    public void setAlphaAssists(double assist) {
        for (Limelight limelight : limelights) {
            LimelightHelpers.SetIMUAssistAlpha(
                    limelight.getId(),
                    assist
            );
        }
    }

    public void updatePositions() {
        for (Limelight limelight : limelights) {
            limelight.updatePosition();
        }
    }

    public void setPipelines(int index) {
        for (Limelight limelight : limelights) {
            LimelightHelpers.setPipelineIndex(
                    limelight.getId(),
                    index
            );
        }
    }

    public void setRewinds(boolean value) {
        for (Limelight limelight : limelights) {
            LimelightHelpers.setRewindEnabled(
                limelight.getId(),
                value
            );
        }
    }

    public void captureRewinds() {
        for (Limelight limelight : limelights) {
            RewindStats stats = LimelightHelpers.getLatestResults(
                    limelight.getId()
                ).rewindStats;

            LimelightHelpers.triggerRewindCapture(
                    limelight.getId(),
                    Math.min(
                        stats.storedSeconds,
                        165
                    )
            );
        }
    }
}
