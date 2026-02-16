package frc.robot.util.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

public abstract class Socket<
    RequestType extends BaseRequestValue
>{

    private final SocketController controller;
    protected final SwerveSubsystem swerve;

    private Command requestingCommand = null;
    private Subsystem requestingSubsystem = null;
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
        if (
            requestingCommand != null
            && (
                requestingCommand.isFinished()
                || !requestingCommand.isScheduled()
            )
            
        ) {
            depossess();
        }

        return possesser != null;
    }

    public RequestType getRequest() {
        if (isPossessed()) {
            return possesser.getRequest();
        } else {
            return null;
        }
    }

    private boolean possess(Requester<RequestType> requester) {
        if (!isPossessed()) {
            possesser = requester;
            return true;
        } else {
            return false;
        }
    }

    public boolean possess(
        Command requestingCommand,
        Requester<RequestType> requester
    ) {
        if (possess(requester)) {
            this.requestingCommand = requestingCommand;
            return true;
        } else {
            return false;
        }
    }

    public boolean possess(
        Subsystem requestingSubsystem,
        Requester<RequestType> requester
    ) {
        if (possess(requester)) {
            this.requestingSubsystem = requestingSubsystem;
            return true;
        } else {
            return false;
        }

    }

    public void depossess(){
        possesser = null;
        requestingCommand = null;
        requestingSubsystem = null;
    }
}
