package frc.robot.commands.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class VoltageStatusCommand extends Command {
    @Override
    public void execute() {
        SmartDashboard.putNumber(
            "Meta/batteryVoltage",
            RobotController.getBatteryVoltage()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}