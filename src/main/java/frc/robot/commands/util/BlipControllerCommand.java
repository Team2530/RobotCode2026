package frc.robot.commands.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BlipControllerCommand extends ParallelRaceGroup {

    /*
     * rumbles the given controller in short bursts
     *
     * @param controller - the controller to rumble
     * @param length - the length of the rumble
     * @param spacing - the delay between bursts
     * @param counts - the number of bursts
     * @param strength - the strength of the bursts; this can be a value between
     * 0 and 1
     */
    public BlipControllerCommand(
        XboxController controller,
        Time length,
        Time spacing,
        int counts,
        double strength
    ) {
        for (int i = 0; i < counts; i++) {
            addCommands(
                new KillRumbleCommand(controller),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        controller.setRumble(
                            RumbleType.kBothRumble,
                            strength
                        );
                    }),
                    new WaitCommand(length),
                    new InstantCommand(() -> {
                        controller.setRumble(
                            RumbleType.kBothRumble,
                            0
                        );
                    }),
                    new WaitCommand(spacing)
                )
            );

        }
    }

    public BlipControllerCommand(XboxController controller) {
        this(
            controller,
            Seconds.of(0.2),
            Seconds.zero(),
            1,
            1
        );
    }

    public BlipControllerCommand(
        XboxController controller,
        int counts
    ) {
        this(
            controller,
            Seconds.of(0.2),
            Seconds.zero(),
            counts,
            1
        );
    }
}
