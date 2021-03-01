package org.firstinspires.ftc.teamcode.drive.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

@TeleOp(name = "SwitchableCameras (Blocks to Java)", group = "TestCode")
@Disabled

public class SwitchableCameras extends LinearOpMode {

  private VuforiaCurrentGame vuforiaUltimateGoal;
  private TfodCurrentGame tfodUltimateGoal;

  boolean oldLeftBumper;
  boolean oldRightBumper;
  Recognition recognition;
  String activeCamera;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    List<Recognition> recognitions;
    double index;

    vuforiaUltimateGoal = new VuforiaCurrentGame();
    tfodUltimateGoal = new TfodCurrentGame();

    // Sample TFOD Op Mode
    // Initialize Vuforia using SwitchableCamera
    vuforiaUltimateGoal.initialize(
        "", // vuforiaLicenseKey
        VuforiaBase.getSwitchableCamera(hardwareMap), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations
    // Set min confidence threshold to 0.7
    tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
    // Initialize TFOD before waitForStart.
    // Init TFOD here so the object detection labels are visible
    // in the Camera Stream preview window on the Driver Station.
    tfodUltimateGoal.activate();
    // Enable following block to zoom in on target.
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    // Wait for start command from Driver Station.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      initCameraSwitching();
      while (opModeIsActive()) {
        // Put loop blocks here.
        doCameraSwitching();
        // Get a list of recognitions from TFOD.
        recognitions = tfodUltimateGoal.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
          telemetry.addData("TFOD", "No items detected.");
        } else {
          index = 0;
          // Iterate through list and call a function to
          // display info for each recognized object.
          for (Recognition recognition : recognitions) {
            // Display info.
            displayInfo(index);
            // Increment index.
            index = index + 1;
          }
        }
        telemetry.update();
      }
    }
    // Deactivate TFOD.
    tfodUltimateGoal.deactivate();

    vuforiaUltimateGoal.close();
    tfodUltimateGoal.close();
  }

  /**
   * Describe this function...
   */
  private void initCameraSwitching() {
    // After Vuforia is initialized, set the
    // active camera to Webcam 1.
    oldLeftBumper = false;
    oldRightBumper = false;
    activeCamera = "Webcam 1";
    vuforiaUltimateGoal.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
  }

  /**
   * Describe this function...
   */
  private void doCameraSwitching() {
    boolean newLeftBumper;
    boolean newRightBumper;

    // If the left bumper is pressed, use Webcam 1.
    // If the right bumper is pressed, use Webcam 2.
    newLeftBumper = gamepad1.left_bumper;
    newRightBumper = gamepad1.right_bumper;
    if (newLeftBumper && !oldLeftBumper) {
      activeCamera = "Webcam 1";
      vuforiaUltimateGoal.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
    } else if (newRightBumper && !oldRightBumper) {
      activeCamera = "Webcam 2";
      vuforiaUltimateGoal.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
    }
    oldLeftBumper = newLeftBumper;
    oldRightBumper = newRightBumper;
    if (activeCamera.equals("Webcam 1")) {
      telemetry.addData("activeCamera", "Webcam 1");
      telemetry.addData("Press RightBumper", "to switch to Webcam 2");
    } else {
      telemetry.addData("activeCamera", "Webcam 2");
      telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
    }
  }

  /**
   * Display info (using telemetry) for a recognized object.
   */
  private void displayInfo(double i) {
    // Display label info.
    // Display the label and index number for the recognition.
    telemetry.addData("label " + i, recognition.getLabel());
    // Display upper corner info.
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData("Left, Top " + i, recognition.getLeft() + ", " + recognition.getTop());
    // Display lower corner info.
    // Display the location of the bottom right corner
    // of the detection boundary for the recognition
    telemetry.addData("Right, Bottom " + i, recognition.getRight() + ", " + recognition.getBottom());
  }
}
