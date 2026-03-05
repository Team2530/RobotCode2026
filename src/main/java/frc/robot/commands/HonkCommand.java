package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HonkCommand extends Command {
    private final Orchestra orchestra;

    public HonkCommand(
        Orchestra orchestra
    ) {
        this.orchestra = orchestra;
    }

    @Override
    public void initialize() {
        orchestra.loadMusic(
            Filesystem.getDeployDirectory() + "/music/honk"
        );

        orchestra.play();
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
