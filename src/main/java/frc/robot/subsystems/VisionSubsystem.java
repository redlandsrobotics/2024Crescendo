package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {}
 

  PhotonCamera camera = new PhotonCamera("RR9615");//Updated name, check for errors though
 
  /**
   * Example command factory method.
   *
   * @return a commandgit 
   */
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void choosePipeline() {
    // if(RobotContainer.joystick1.getRawButtonReleased(10)){
    //   camera.setPipelineIndex(0);//reflective tape
    // }
    // if(RobotContainer.joystick1.getRawButtonReleased(9)){
    //   camera.setPipelineIndex(1);//April tag
    // }
    // if(RobotContainer.joystick1.getRawButtonReleased(7)){
    //   camera.setPipelineIndex(2);//Cube
    // }
    // if(RobotContainer.joystick1.getRawButtonReleased(8)){
    //   camera.setPipelineIndex(3);//Cone
    camera.setDriverMode(true);

  }


  public double[] see(){
    var result = camera.getLatestResult(); // find latest result from camera
    boolean hasTargets = result.hasTargets(); // check if camera has any targets

    List<PhotonTrackedTarget> targets = result.getTargets(); // get list of tracked targets
    PhotonTrackedTarget target = result.getBestTarget(); // get best target

    double yaw = 0.0;
    double pitch = 0.0;
    double area = 0.0;
    double skew = 0.0;
    int targetID = 0;
    double poseAmbiguity = 0.0;
    Transform3d bestCameraToTarget = null; 

    if (hasTargets){
      yaw = target.getYaw(); // get yaw
      pitch = target.getPitch(); // get pitch
      area = target.getArea(); // get area of target
      skew = target.getSkew(); // figure out how parallel camera is to target
     List<TargetCorner> corners = target.getDetectedCorners(); // get all detected corners of target

      targetID = target.getFiducialId(); // get ID of april tag target
      poseAmbiguity = target.getPoseAmbiguity(); // Used to find out how ambiguous target is. If -1, target is invalid. Ambiguous targets generally > 0.2
      bestCameraToTarget = target.getBestCameraToTarget(); // gets transform from camera distance to target
     Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget(); // find transform in any other alternate ways
       
    } 


    // System.out.println("April Tag Id: " + targetID);
    // System.out.println(" ");
    // System.out.println("Has Targets (T/F): " + hasTargets);
    // System.out.println(" ");
    // System.out.println("Yaw: " + yaw);
    // System.out.println(" ");
    // System.out.println("Transform 3d: " + bestCameraToTarget);
    double[] values = {yaw,area};

    return values;
  }

  public boolean hasTargets() {
    var result = camera.getLatestResult(); // find latest result from camera
    boolean hasTargets = result.hasTargets(); // check if camera has any targets

    return hasTargets;
  }

  @Override
  public void periodic() {
    //see();
  }



    // returns the transformation from camera to target
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
 