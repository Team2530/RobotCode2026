package frc.robot.util.swerve;

public interface Requester<
    RequestValue extends BaseRequestValue
> {
    public boolean isRequestingActive();
    public RequestValue getRequest();
}
