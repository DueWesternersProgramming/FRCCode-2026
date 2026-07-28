package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionSubsystemConstants.VisionSource;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.TimestampedPose;

import java.util.List;
import java.util.Optional;

public class TurretedAprilTagCameraIOPhoton implements AprilTagCameraIO {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    private int servoPort;
    private Servo servo;
    private VisionSource source;

    public TurretedAprilTagCameraIOPhoton(VisionSource source, int servoPort) {
        this.servoPort = servoPort;
        this.source = source;

        photonCamera = new PhotonCamera(source.name());

        photonPoseEstimator = new PhotonPoseEstimator(
                CowboyUtils.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                source.robotToCamera());
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        
        servo = new Servo(servoPort);
        servo.setAngle(90);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        inputs.mounting = photonPoseEstimator.getRobotToCameraTransform();
        inputs.connected = photonCamera.isConnected();
        inputs.targetIDs = photonCamera.getLatestResult().getTargets().stream()
                .map(PhotonTrackedTarget::getFiducialId)
                .mapToInt(Integer::intValue)
                .toArray();
        inputs.hasTargets = photonCamera.getLatestResult().hasTargets();

        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults(); // Call this only once!!

        if (!results.isEmpty()) {            
            try {

                if (results.size() > 0) {
                    for (PhotonPipelineResult result : results) {
                        if (result.hasTargets()) {
                            PhotonTrackedTarget target = result.getBestTarget();

                            inputs.bestTargetArea = target.getArea();

                            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);
                            estimatedRobotPose.ifPresent(est -> {
                                Pose2d pose = estimatedRobotPose.get().estimatedPose.toPose2d();
                                inputs.pose = pose;
                                RobotState.offerAprilTagCameraMeasurement(
                                        new TimestampedPose(pose,
                                                estimatedRobotPose.get().timestampSeconds));
                            });

                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

    }

    @Override
    public List<PhotonPipelineResult> getResult() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return photonCamera.getAllUnreadResults();

        } else {
            return null;
        }
    }

    @Override
    public boolean isCameraConnected() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return photonCamera.isConnected();
        } else {
            return false;
        }
    }

    @Override
    public PhotonTrackedTarget getBestTarget() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return getResult().get(0).getBestTarget();
        } else {
            return null;
        }
    }

    @Override
    public double getBestTargetYaw() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return getBestTarget().getYaw();
        } else {
            return 0;
        }
    }

    @Override
    public void setCameraTransformOffset(Transform3d offset){
        photonPoseEstimator.setRobotToCameraTransform(offset);
    }

    public void setTurretPosition(double angle){
        servo.setAngle(angle);
        setCameraTransformOffset(source.robotToCamera().plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, angle))));
    }

}
