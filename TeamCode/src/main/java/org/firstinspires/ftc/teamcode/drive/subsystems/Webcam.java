package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.opmodes.Calibration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Webcam {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    protected Util util;
    protected Lights lights;

    private static final Integer webcamSamples      = 5;
    private static final String TFOD_MODEL_ASSET    = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String vuforiaLicenseKey   = "ARXKY4r/////AAABmb9ScOj+l0t0ira7sOidsLBSfpaAW93Eice6VSZw+/6407uwU19CCmRcgiwVSM/jPNg0xidl+w2MSEiJIun4VopbwVTeTZ/g0drowWZfIqxZQyEjW+xE5+rZ5JFf9X5tAkRmEhVyGu5F1luxs1RlHh2VkNxu8h7aQV98izYxc4FmvKCqTPB/otF3ncjr1YgyQGKGT6WXeDAXb1FRxlBQ8eETyqJWLWswZOW5yLyXKb2ufZz+9JevXOzJIbAuECTdXgJiQolYQvGzY+zLM00hxRpKj7M7buHEqCFKque/yotyYCGlTX5RgNuGMEDKN2lIxhyR7dYvLwarfWmHRnyCR2C9CboRWy9pGgwCtQrf1gnj";
    private static final float  minResultConfidence = 0.6f;

    protected VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfObjectDetector;
    protected VuforiaLocalizer.Parameters localizerParameters;
    VuforiaCurrentGame vuforiaUltimateGoal;
    TfodCurrentGame tfodUltimateGoal;
    //Recognition recognition;

    protected OpenGLMatrix lastLocation = null;
    public TensorFlowDetectedObject tensorFlowDetectedObject = new TensorFlowDetectedObject();

    public boolean initialized = false;

    public static class TensorFlowDetectedObject {
        public String label = "";
        public double left,top,right,bottom;
    }

    public Webcam(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        this.util = robotBase.util;
        this.lights = robotBase.lights;
        initHardware();
    }

    protected void initHardware() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        /* This is a universal configuration that works for both getVuforiaLocalization and
           getTensorFlowObjectDetection. However it rezstricts the view to only a small rectangle
           at the center of the screen. Nore research is needed to figure out why this is. TODO
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //localizerParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //localizerParameters.vuforiaLicenseKey = vuforiaLicenseKey;
        //localizerParameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //localizerParameters.useExtendedTracking = false;
        //localizerParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //vuforia = ClassFactory.getInstance().createVuforia(localizerParameters);

        /*
         * This is a game specific initialization, need to figure out what is going on in the background here. TODO
         */
        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();
        vuforiaUltimateGoal.initialize(vuforiaLicenseKey, hardwareMap.get(WebcamName.class, "Webcam 1"), "", false, // useExtendedTracking
                false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,0, 0, 0, 0, 0, 0, true);
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, minResultConfidence, true, true);
        tfodUltimateGoal.activate();
        initialized = true;
    }


    public void getVuforiaLocalization () {
        int sampleCount = 0;
        /**
         * This uses the Vuforia localizer to determine
         * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
         * The code is structured as a LinearOpMode
         *
         * When images are located, Vuforia is able to determine the position and orientation of the
         * image relative to the camera.  This code then combines that information with a
         * knowledge of where the target images are on the field, to determine the location of the camera.
         *
         * From the Audience perspective, the Red Alliance station is on the right and the
         * Blue Alliance Station is on the left.

         * There are a total of five image targets for the ULTIMATE GOAL game.
         * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
         * and Blue Alliance perimeter walls.
         * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
         * Refer to the Field Setup manual for more specific location details
         *
         * A final calculation then uses the location of the camera on the robot to determine the
         * robot's location and orientation on the field.
         */

        // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        final float mmPerInch   = 25.4f;
        float mmTargetHeight    = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Constants for perimeter targets
        float halfField         = 72 * mmPerInch;
        float quadField         = 36 * mmPerInch;

        boolean targetVisible = false;
        float phoneXRotate      = 0;
        float phoneYRotate      = 0;
        float phoneZRotate      = 0;

        // Load the data sets for the trackable objects. These particular data sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Create a transformation matrix describing where the camera is on the robot.
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The camera configuration starts out lying flat, with the screen facing Up and with the physical top of the camera
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        phoneYRotate = -90;

        // Rotate the camera vertical about the X axis if it's in portrait mode
        phoneXRotate = 90 ;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, localizerParameters.cameraDirection);
        }

        targetsUltimateGoal.activate();
        initialized = true;

        while (initialized && sampleCount <= webcamSamples) {
            sampleCount++;

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    public void getTensorFlowObjectDetection() {
        int sampleCount = 0;

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = minResultConfidence;
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfObjectDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfObjectDetector != null) {
            tfObjectDetector.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfObjectDetector.setZoom(2.5, 16.0/9.0);
        }

        while (initialized && sampleCount <= webcamSamples) {
            sampleCount++;
            List<Recognition> recognitions;

            //Arena area is sometimes dark, make sure the lights are on, and give them time to update
            lights.setPattern("WHITE");
            util.sleep(Calibration.BLINKIN_UPDATE_LOCKOUT);

            //Get a list of recognitions from TFOD.
            recognitions = tfObjectDetector.getUpdatedRecognitions();

            // If list is empty, label for user. Otherwise, go through list and display info for each recognition.
            if (recognitions.size() == 0) {
                tensorFlowDetectedObject.label = "NONE";
            } else {
                //Found one or more objects, iterate through list and call a function to record info for each recognized object.
                //@TODO - this only records the last found object, should we check that they all match?
                for (Recognition recognition : recognitions) {
                    tensorFlowDetectedObject.label = recognition.getLabel();
                    tensorFlowDetectedObject.left = recognition.getLeft();
                    tensorFlowDetectedObject.top = recognition.getTop();
                    tensorFlowDetectedObject.right = recognition.getRight();
                    tensorFlowDetectedObject.bottom = recognition.getBottom();
                }
                break;
            }
        }


        telemetry.addData("TFOD:", tensorFlowDetectedObject.label);
        telemetry.update();
        lights.setPattern("BLACK");
        if (tfObjectDetector != null) {
            tfObjectDetector.deactivate();
        }

    }

    public void getTensorFlowObjectDetectionCurrentGame () {
        List<Recognition> recognitions;
        int sampleCount = 0;

        while (initialized && sampleCount <= webcamSamples) {
            sampleCount++;

            //Arena area is sometimes dark, make sure the lights are on, and give them time to update
            lights.setPattern("WHITE");
            util.sleep(Calibration.BLINKIN_UPDATE_LOCKOUT);

            //Get a list of recognitions from TFOD.
            recognitions = tfodUltimateGoal.getRecognitions();

            // If list is empty, label for user. Otherwise, go through list and display info for each recognition.
            if (recognitions.size() == 0) {
                tensorFlowDetectedObject.label = "NONE";
            } else {
                //Found one or more objects, iterate through list and call a function to record info for each recognized object.
                //@TODO - this only records the last found object, should we check that they all match?
                for (Recognition recognition : recognitions) {
                    tensorFlowDetectedObject.label = recognition.getLabel();
                    tensorFlowDetectedObject.left = recognition.getLeft();
                    tensorFlowDetectedObject.top = recognition.getTop();
                    tensorFlowDetectedObject.right = recognition.getRight();
                    tensorFlowDetectedObject.bottom = recognition.getBottom();
                }
                break;
            }
        }

        // Deactivate TFOD.
        telemetry.addData("TFOD:", tensorFlowDetectedObject.label);
        telemetry.update();
        lights.setPattern("BLACK");
        if (tfodUltimateGoal != null) {
            tfodUltimateGoal.deactivate();
            tfodUltimateGoal.close();
        }
        if (vuforiaUltimateGoal != null) {
            vuforiaUltimateGoal.close();
        }
    }

    public void stop() {
        if (initialized && tfObjectDetector != null) {
            tfObjectDetector.shutdown();
            initialized = false;
        }
    }

    public void reset() {
    }
}
