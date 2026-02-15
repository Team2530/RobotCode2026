package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class TranslationSocket extends Socket {

    public TranslationSocket(
        SocketController controller,
        SwerveSubsystem swerve
    ) {
        super(
            controller,
            swerve
        );
    }

    private interface TranslationRequest extends BaseRequestValue<
        DriveMode,
        Translation2d
    > {};
    
    public class RelativeTranslationRequest
        implements TranslationRequest
    {
        private final Translation2d relative;
        public RelativeTranslationRequest(Translation2d relative)
            { this.relative = relative; }

        public DriveMode getMode()
            { return DriveMode.RELATIVE; }

        public Translation2d getUsableValue() {
            return relative.rotateBy(
                swerve.getRotation().toRotation2d()
            );
        }
    }

    public class AbsoluteTranslationRequest
            implements TranslationRequest
    {
        private final Translation2d absolute;
        public AbsoluteTranslationRequest(Translation2d absolute)
            { this.absolute = absolute; }

        public DriveMode getMode()
            { return DriveMode.ABSOLUTE; }

        public Translation2d getUsableValue() {
            return absolute;
        }
    }

    public class ToPointTranslationRequest
        implements TranslationRequest
    {
        private final Pose2d point;
        public ToPointTranslationRequest(Pose2d point)
            { this.point = point; }

        public DriveMode getMode()
            { return DriveMode.TO_POINT; }

        // TODO: there's a smoother way to do this
        public Translation2d getUsableValue() {
            return swerve.getPose()
                .minus(point)
                .getTranslation();
        }
    }
}
