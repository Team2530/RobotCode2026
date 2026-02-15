package frc.robot.util.swerve;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SocketController;

public abstract class Socket {

    private final SocketController controller;
    protected final SwerveSubsystem swerve;

    public Socket(
        SocketController controller,
        SwerveSubsystem swerve
    ) {
        this.controller = controller;
        this.swerve = swerve;
    }

    public boolean isActive() {
        return controller.getActiveSocket()
            .equals(this);
    };
}
