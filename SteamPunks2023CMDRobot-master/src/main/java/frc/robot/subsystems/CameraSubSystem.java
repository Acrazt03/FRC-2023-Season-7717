package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Values;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class CameraSubSystem extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("OV5647");

  PhotonPipelineResult result;

  PhotonTrackedTarget bestTarget;

  private double distance = 0f;

  public CameraSubSystem() {
  } 

  @Override
  public void periodic() {
    readTargets();
    calculateDistance();
  }

  private void readTargets(){
    result = camera.getLatestResult();
  }

  public boolean isThereATarget(){
    boolean hasTargets = result.hasTargets();
    return hasTargets;
  }

  private void calculateDistance(){
    if(isThereATarget()){

      bestTarget = result.getBestTarget();

      SmartDashboard.putNumber("Detected AprilTag ID: ", bestTarget.getFiducialId());
      distance = PhotonUtils.calculateDistanceToTargetMeters(
                                  Values.Constants.CAMERAHMETERS,
                                  Values.Constants.TARGETHMETERS,
                                  Units.degreesToRadians(Values.Constants.CAMERAPITCH),
                                  Units.degreesToRadians(-bestTarget.getPitch()));
      
      SmartDashboard.putNumber("Distance to target: ", distance);
    }
  }

  public double getYaw(){
    if(isThereATarget()){
      return bestTarget.getYaw();
    }else{
      return 0;
    }
  }

  public double getDistance(){
    return distance;
  }

}