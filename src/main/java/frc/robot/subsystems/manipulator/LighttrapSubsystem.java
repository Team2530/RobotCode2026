package frc.robot.subsystems.manipulator;

import java.util.ArrayDeque;
import java.util.Queue;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LighttrapSubsystem extends SubsystemBase {
    
    private final int startTrapPin = 0;
    private final int endTrapPin = 1;

    private boolean previouslyStartTrapped = true;
    private boolean previouslyEndTrapped = true;

    private Queue<Double> timestamps = new ArrayDeque<>();

    public LighttrapSubsystem() {}

    @Override
    public void periodic() {
        if (
            DIOJNI.getDIO(startTrapPin)
            && !previouslyStartTrapped
        ) {
            previouslyStartTrapped = true;
            double time = Timer.getTimestamp();
            timestamps.add(
                time
            );
            SmartDashboard.putNumber(
                "Lighttrap/lastEntry",
                time
            );
        } else {
            previouslyStartTrapped = false;
        }

        if (
            DIOJNI.getDIO(endTrapPin)
            && !previouslyEndTrapped
        ) {
            double time = Timer.getTimestamp();
            previouslyEndTrapped = true;
            SmartDashboard.putNumber(
                "Lighttrap/lastDifference",
                Timer.getTimestamp() - timestamps.remove()
            );
            SmartDashboard.putNumber(
                "Lighttrap/lastExit",
                time
            );
        } else {
            previouslyEndTrapped = false;
        }

        SmartDashboard.putBoolean(
            "Lightrap/start_trapped",
            DIOJNI.getDIO(startTrapPin)
        );
        SmartDashboard.putBoolean(
            "Lightrap/end_trapped",
            DIOJNI.getDIO(endTrapPin)
        );
    }

}
