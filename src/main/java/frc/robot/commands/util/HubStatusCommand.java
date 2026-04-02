package frc.robot.commands.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.MetaConstants;

public class HubStatusCommand extends Command {
    @Override
    public void execute() {
        SmartDashboard.putBoolean(
            "Meta/Shifts/hubIsActive",
            MetaConstants.Game.Timing.isActive.getAsBoolean()
        );
        SmartDashboard.putString(
            "Meta/Shifts/ShiftType",
            MetaConstants.Game.Timing.getCurrentShift().toString()
        );
        SmartDashboard.putNumber(
            "Meta/Shifts/timeTillChange",
            MetaConstants.Game.Timing.timeTillShiftChange().in(Seconds)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
