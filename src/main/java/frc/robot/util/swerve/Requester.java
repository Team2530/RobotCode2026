package frc.robot.util.swerve;

import frc.robot.util.swerve.BaseRequestValue;

public interface Requester<
    RequestValue extends BaseRequestValue
> {
    public boolean requestingActive();
    public RequestValue getRequest();
}
