package frc.robot.subsystems.manipulator;

import java.util.ArrayDeque;
import java.util.Queue;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LighttrapSubsystem extends SubsystemBase {
    
    private final DigitalInput startTrap = new DigitalInput(3);
    private final DigitalInput endTrap = new DigitalInput(2);

    private boolean previouslyStartTrapped = true;
    private boolean previouslyEndTrapped = true;

    private Queue<Double> timestamps = new ArrayDeque<>();

    public LighttrapSubsystem() {}

    @Override
    public void periodic() {
        if (
            !startTrap.get()
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
            !endTrap.get()
            && !previouslyEndTrapped
        ) {
            double time = Timer.getTimestamp();
            previouslyEndTrapped = true;


            if (timestamps.size() > 0) {

                double PreviousTime = timestamps.remove();

                SmartDashboard.putNumber(
                    "Lighttrap/lastDifference",
                    Timer.getTimestamp() - PreviousTime
                );

                SmartDashboard.putNumber(
                    "Lighttrap/PreviousTime",
                    PreviousTime
                );
            }
            SmartDashboard.putNumber(
                "Lighttrap/lastExit",
                time
            );

        } else {
            previouslyEndTrapped = false;
        }

        SmartDashboard.putNumber(
            "Lighttrap/queue_size",
            timestamps.size()
        );
        

        SmartDashboard.putBoolean(
            "Lighttrap/start_trapped",
            startTrap.get()
        );
        SmartDashboard.putBoolean(
            "Lighttrap/end_trapped",
            endTrap.get()
        );

    }

    public void clearQueue() {
        timestamps.clear();
    }


}
