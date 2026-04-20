package frc.robot.subsystems.limelight;

import static frc.robot.util.LimelightHelpers.PoseEstimate;


public class Reading {
    public enum ReadingType {
        MT1,
        MT2
    }

    private final ReadingType type;
    private final PoseEstimate estimate;
    private final double[] stddevs;
    private final int imuMode;

    public Reading(
        ReadingType type,
        PoseEstimate estimate,
        double[] stddevs,
        int imuMode
    ) {
        this.type = type;
        this.estimate = estimate;
        this.stddevs = stddevs;
        this.imuMode = imuMode;
    }

    public ReadingType getType() {
        return type;
    }

    public PoseEstimate getEstimate() {
        return estimate;
    }

    public double[] getStddevs() {
        return stddevs;
    }

    public int getIMUMode() {
        return imuMode;
    }
}
