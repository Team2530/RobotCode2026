package frc.robot.subsystems.limelight;

import static edu.wpi.first.units.Units.*;

import frc.robot.RobotContainer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{

    private final ArrayList<Limelight> limelights;

    public LimelightSubsystem(Limelight... limelights) {
       this.limelights = new ArrayList<>(
               Arrays.asList(limelights)
           );
    }

    @Override
    public void periodic() {
        if (
            RobotContainer.swerveDriveSubsystem.getAngularVelocity()
                .abs(DegreesPerSecond)
            > 2
        ) {
            setIMUModes(4);
            setAlphaAssists(0.01);
            
        } else {
            setIMUModes(4);
            setAlphaAssists(0.03);
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
            limelight.setIMUMode(mode);
        }
    }

    public void setAlphaAssists(double assist) {
        for (Limelight limelight : limelights) {
            limelight.setAlphaAssist(assist);
        }
    }

    public void updatePositions() {
        for (Limelight limelight : limelights) {
            limelight.updatePosition();
        }
    }
}
