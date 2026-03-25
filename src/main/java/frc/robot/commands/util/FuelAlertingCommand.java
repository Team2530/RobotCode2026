package frc.robot.commands.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem.IndexerFuelState;

import static frc.robot.RobotContainer.indexerSubsystem;

public class FuelAlertingCommand extends Command {
    
    private final XboxController controller;

    public FuelAlertingCommand(
        XboxController controller
    ) {
        this.controller = controller;
    }

    @Override
    public void execute() {
        double rumbleStrength = indexerSubsystem.getFuelState()
                == IndexerFuelState.FUELED
            ? 0.5
            : 0;

        controller.setRumble(
            RumbleType.kBothRumble,
            rumbleStrength
        );
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(
            RumbleType.kBothRumble,
            0
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
