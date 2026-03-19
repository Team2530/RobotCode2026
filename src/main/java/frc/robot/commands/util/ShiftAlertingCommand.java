package frc.robot.commands.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShiftAlertingCommand extends ParallelCommandGroup {
    
    public ShiftAlertingCommand(
        BlipControllerCommand blipCommand,
        Time alertHead
    ) {
        Time[] timings = {
            Seconds.of(30),
            Seconds.of(55),
            Seconds.of(80),
            Seconds.of(105),
            Seconds.of(130)
        };

        for (Time timing : timings) {
            addCommands(
                new SequentialCommandGroup(
                    new WaitCommand(
                        timing.minus(alertHead)
                    ),
                    blipCommand
                )
            );
        }
    }

    public ShiftAlertingCommand(
        XboxController controller,
        Time alertHead
    ) {
        this(
            new BlipControllerCommand(
                controller,
                2
            ),
            alertHead
        );
    }
}
