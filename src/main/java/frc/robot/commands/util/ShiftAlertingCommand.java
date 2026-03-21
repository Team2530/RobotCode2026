package frc.robot.commands.util;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MetaConstants;

public class ShiftAlertingCommand extends ParallelCommandGroup {
    
    public ShiftAlertingCommand(
        XboxController controller,
        Time alertHead
    ) {
        for (Time timing : MetaConstants.Game.Timing.SHIFT_TIMINGS) {
            addCommands(
                new SequentialCommandGroup(
                    new WaitCommand(
                        timing.minus(alertHead)
                    ),
                    new BlipControllerCommand(
                        controller,
                        2
                    )
                )
            );
        }
    }
}
