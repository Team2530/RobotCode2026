package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {
    public enum LimelightType {
        // LL1 doesn't have specs listed, so these could be incorrect
        LL1(54, 41),
        LL2(62.5, 48.9),
        LL2Plus(62.5, 48.9),
        LL3(62.5, 48.9),
        LL3G(82, 56.2),
        LL4(82, 56.2);

        private double HFOV;
        private double VFOV;

        private LimelightType(double HFOV, double VFOV) {
            this.HFOV = HFOV;
            this.VFOV = VFOV;
        }
    }

    private final LimelightType limelightType;
    private final String name;
    private boolean isEnabled;
    private boolean cropEnabled;
    private double lastPoseEstimate = 0;
    private int counter = 0;
    private double lastFrame = 0;

    private final StructPublisher<Pose2d> publisher;


    public Limelight(LimelightType limelightType, String name, boolean isEnabled, boolean cropEnabled) {
        this.limelightType = limelightType;
        this.name = name;
        this.isEnabled = isEnabled;
        this.cropEnabled = cropEnabled;

        publisher =  NetworkTableInstance.getDefault().getStructTopic(name, Pose2d.struct).publish();

        LimelightHelpers.SetIMUMode(name, 1);
    }

    @Override
    public void periodic() {
        if (isEnabled) {
            if (cropEnabled) {
                if (numTargets() > 0) {
                    smartCrop();
                } else {
                    restoreCrop();
                }
            }
        }
    }

    public int numTargets() {
        return LimelightHelpers.getRawFiducials(name).length;
    }

    public void smartCrop() {
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if(poseEstimate == null) {
            return;
        }

        if (lastPoseEstimate != poseEstimate.timestampSeconds){
            counter = ++counter % 50;
            if(counter > 40){ // For every 5 frames, out of 50, check the entire screen for apriltags
                restoreCrop(); // 45 will be cropped onto whichever limelights they find
                return;
            }
            
            lastPoseEstimate = poseEstimate.timestampSeconds; //making sure we're only running this process on new frames
            RawFiducial[] targets = LimelightHelpers.getRawFiducials(name); //get our targets
            boolean useCrop = true; //we are currently using it, more changes will be implemented
            Translation2d[] targetTranslations = new Translation2d[targets.length]; 
            double area = 0;

            for (int i = 0; i < targetTranslations.length; ++i) {
                targetTranslations[i] = new Translation2d(targets[i].txnc, targets[i].tync); 
                area = Math.max(area, targets[i].ta); //Finding the largest area of the targets
            }

            double xc = 0, yc = 0;

            for (int i = 0; i < targetTranslations.length; ++i) { // Add to the width, depending on the locations of the other targets
                xc += targetTranslations[i].getX(); //If one is at (2, 1), the width of our box increases on each size by 2, and the height by 1
                yc += targetTranslations[i].getY(); //xc & xy increase per target
            }

            xc /= targetTranslations.length; //Then, average depending on the number of targets
            yc /= targetTranslations.length; // (2,1), (.05, .25), (-1, 2) -> xc =.35, yc= .75

            xc = xc / (limelightType.HFOV / 2); // scales it to FOV
            yc = yc / (limelightType.VFOV / 2);

            double borderx = 0, bordery = 0; 

            if (targetTranslations.length > 1) { // for more than one tag
                borderx = (area + 0.75) * 0.22 * targetTranslations.length + .2; // relatively randomly generated border
                bordery = (area + 0.5) * 0.22 * targetTranslations.length;
            } 

            //todo optimize

            else {
                if (area > 0.75){
                    borderx = 1;
                    bordery = .5;
                }
                else if (area > 0.5) {  //Remember that "area" only refers to the area of the largest apriltag
                    borderx = 0.8; 
                    bordery = 0.5;
                } 
                else if (area > .015){ 
                    borderx = 1; 
                    bordery = 0.25;
                }
                else if (area > .005){
                    borderx = 1;
                    bordery = .5;
                }
                else { 
                    borderx = 1;
                    bordery = .5;
                }
            }


            // Doesn't let borders extend beyond -1, 1 for x & y
            double xlim = (xc - borderx < -1) ? -1 : xc - borderx;
            double xlim2 = (xc - borderx > 1) ? 1 : xc + borderx;
            double ylim = (yc - bordery < -1) ? -1 : yc - borderx;
            double ylim2 = (yc - bordery > 1) ? 1 : xc + borderx;

            if (useCrop) {
                LimelightHelpers.setCropWindow(name, xlim, xlim2, ylim, ylim2);
                // Finds the center of the targets, tries to build a big enough box from there
            }
        }
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(name, mode);
    }

    public void restoreCrop() {
        LimelightHelpers.setCropWindow(name, -1, 1, -1, 1);
    }

    
    public void setEnabled(boolean enabled) {
        this.isEnabled = enabled;
    }

    public void setCropEnabled(boolean enabled) {
        this.cropEnabled = enabled;
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    public boolean isCropEnabled() {
        return cropEnabled;
    }

    public double getLastFrameTime(){
        return lastFrame;
    }

    public void setLastFrame(double lastFrameTime){
        lastFrame = lastFrameTime;
    }

    public double getVFOV() {
        return limelightType.VFOV;
    }

    public double getHFOV() {
        return limelightType.HFOV;
    }

    public LimelightType getLimelightType() {
        return limelightType;
    }

    public String getName() {
        return name;
    }


  public void pushPoseToShuffleboard(String name, Pose2d pose) {
    publisher.set(pose);
  }

}