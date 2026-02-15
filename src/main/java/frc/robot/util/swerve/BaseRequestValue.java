package frc.robot.util.swerve;

public interface BaseRequestValue<
    ControlMode,
    UsableType
> {
    public ControlMode getMode();
    public UsableType getUsableValue();
}
