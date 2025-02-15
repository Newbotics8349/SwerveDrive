package frc.robot.AprilTagTracking;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;


public class AprilTag {
    private int id;
    private double yaw;
    private double pitch;
    private double area;
    private Transform3d transform;
    
    /**
     * Builds the april tag target from PhotonVision data
     * @param target, photonvision object
     */
    AprilTag(PhotonTrackedTarget target)
    {
        id = target.getFiducialId();
        yaw = target.getYaw();
        pitch = target.getPitch();
        area = target.getArea();
        transform = target.getBestCameraToTarget();
    }


    /**
     * Accessor for target yaw angle.
     * @return The yaw angle of the target
     */
    public double GetYaw()
    {
        return yaw;
    }

    /**
     * Accessor for target pitch angle.
     * @return The pitch angle of the target
     */
    public double GetPitch()
    {
        return pitch;
    }

    public int GetId()
    {
        return id;
    }

    public Transform3d GetTransform()
    {
        return transform;
    }

}
