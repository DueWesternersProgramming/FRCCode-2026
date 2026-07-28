package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.subsystems.vision.VisionSubsystemConstants.VisionSource;
import frc.robot.utils.CowboyUtils;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class AprilTagCameraIOPhotonSim extends AprilTagCameraIOPhoton {
    private final PhotonCameraSim cameraSim;

    public AprilTagCameraIOPhotonSim(
            VisionSource source,
            SimCameraConfig config) {
        super(source); // Explicitly call the superclass constructor

        SimCameraProperties props = config.apply(new SimCameraProperties());

        cameraSim = new PhotonCameraSim(photonCamera, props, CowboyUtils.aprilTagFieldLayout);

        cameraSim.enableDrawWireframe(true);
        cameraSim.setMaxSightRange(10.0);
        cameraSim.setWireframeResolution(1);

        VisionSubsystemConstants.aprilTagSim.ifPresent(
                aprilTagSim -> aprilTagSim.addCamera(cameraSim, source.robotToCamera()));
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        super.updateInputs(inputs);

        VisionSubsystemConstants.aprilTagSim.ifPresent(
                aprilTagSim -> {
                    FieldObject2d visionEstimation = aprilTagSim.getDebugField().getObject("VisionEstimation");

                    visionEstimation.setPoses(visionEstimation.getPoses());
                });
    }
}