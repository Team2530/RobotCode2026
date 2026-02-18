package frc.robot.util.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.SwerveSubsystem.SteerMode;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;

public class RotationSocket
    extends Socket<RotationSocket.RotationRequest>
{

    public RotationSocket(
        SocketController controller,
        SwerveSubsystem swerve
    ) {
        super(
            controller,
            swerve
        );
    }

    // probby not the greatest pattern but whatever
    public interface RotationRequest
       extends BaseRequestValue<
            SteerMode,
            Rotation2d
        >
    {};

    public class RelativeRotationRequest
        implements RotationRequest
    {
        private final Rotation2d relative;
        public RelativeRotationRequest(Rotation2d relative)
            { this.relative = relative; }

        public SteerMode getMode() 
            { return SteerMode.RELATIVE; }

        public Rotation2d getUsableValue() {
            return relative;
        }
    }

    public class AbsoluteRotationRequest
        implements RotationRequest
    {
        private static final PIDController rotationPID = new PIDController(
            DriveConstants.Sockets.Rotation.HeadingPID.P,
            DriveConstants.Sockets.Rotation.HeadingPID.I,
            DriveConstants.Sockets.Rotation.HeadingPID.D
        );

        private final Rotation2d absolute;
        public AbsoluteRotationRequest(Rotation2d absolute)
            { this.absolute = absolute; }

        public SteerMode getMode()
            { return SteerMode.ABSOLUTE; }

        public Rotation2d getUsableValue() {
            return new Rotation2d(
                rotationPID.calculate(
                    swerve.getRotation()
                        .toRotation2d()
                        .getRadians(),
                    absolute.getRadians()
                )
            );
        }
    }


    public class ToPointRotationRotationRequest
        implements RotationRequest
    {
        private static final PIDController rotationPID = new PIDController(
            DriveConstants.Sockets.Rotation.HeadingPID.P,
            DriveConstants.Sockets.Rotation.HeadingPID.I,
            DriveConstants.Sockets.Rotation.HeadingPID.D
        );

        private final Pose2d point;
        public ToPointRotationRotationRequest(Pose2d point)
            { this.point = point; }

        public SteerMode getMode()
            { return SteerMode.TO_POINT; }

        public Rotation2d getUsableValue() {
            return new Rotation2d(
                rotationPID.calculate(
                    swerve.getRotation()
                        .toRotation2d()
                        .getRadians(),
                    swerve.getPose()
                        .relativeTo(point)
                        .getTranslation()
                        .getAngle()
                        .getRadians()
                )
            );
        }
    }
}
