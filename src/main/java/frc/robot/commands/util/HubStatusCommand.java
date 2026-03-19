package frc.robot.commands.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.MetaConstants;

public class HubStatusCommand extends Command {
    @Override
    public void execute() {
        SmartDashboard.putBoolean(
            "Meta/hubIsActive",
            MetaConstants.isActive.getAsBoolean()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
