package frc.robot.util.swerve;

import frc.robot.subsystems.SwerveSubsystem;

public abstract class Socket<
    RequestType extends BaseRequestValue
>{

    private final SocketController controller;
    protected final SwerveSubsystem swerve;

    private Requester<RequestType> possesser = null;

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

    public boolean isRequestingActive() {
        return (
            possesser != null
            && possesser.isRequestingActive()
        );
    }

    public boolean isPossessed() {
        return possesser != null;
    }

    public RequestType getRequest() {
        if (isPossessed()) {
            return possesser.getRequest();
        } else {
            return null;
        }
    }

    public boolean possess(Requester<RequestType> requester) {
        if (!isPossessed()) {
            possesser = requester;
            return true;
        } else {
            return false;
        }
    }
}
