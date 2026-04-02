package frc.robot.commands.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class KillRumbleCommand extends Command {
    
    private XboxController controller;

    public KillRumbleCommand(
        XboxController controller
    ) {
        this.controller = controller;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }
}
