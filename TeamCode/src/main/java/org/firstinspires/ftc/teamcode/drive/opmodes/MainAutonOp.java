package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.*;

import static org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.State.*;

//@Autonomous(name="Autonomous", group = "auto")
@Disabled
public class MainAutonOp extends RobotBase {

    public enum State {
        BEGIN,
        READ_WEBCAM,
        GET_AUTO_PATHS,
        DRIVE_TO_WOBBLE,
        DROP_WOBBLE,
        DRIVE_TO_SHOOT,
        SHOOT,
        DRIVE_TO_PARK,
        PARK,
        IDLE
    }
    public State currentState = BEGIN;

    public enum StartPosition {
        NONE,
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public StartPosition startPosition = StartPosition.NONE;

    public enum PathName {
        BLUE_LEFT_QUAD,BLUE_LEFT_SINGLE,BLUE_LEFT_NONE,
        BLUE_RIGHT_QUAD,BLUE_RIGHT_SINGLE,BLUE_RIGHT_NONE,
        RED_LEFT_QUAD,RED_LEFT_SINGLE,RED_LEFT_NONE,
        RED_RIGHT_QUAD,RED_RIGHT_SINGLE,RED_RIGHT_NONE,
        RED_GO_SHOOT, BLUE_GO_SHOOT
    }
    public PathName pathName;
    public AutoPath autoPath;
    public SampleMecanumDrive rrdrive;

    //Timers
    private final ElapsedTime time = new ElapsedTime();
    private long iterationCounter = 0;
    double dropWobbleTime = 2.0;
    ElapsedTime dropWobbleTimer = new ElapsedTime();
    double shooterTriggerTime = 3.2;
    ElapsedTime shooterTriggerTimer = new ElapsedTime();
    double parkTime = 2.0;
    ElapsedTime parkTimer = new ElapsedTime();

    /**
     * Constructor
     */
    public MainAutonOp() {
    }

    @Override
    public void init() {
        super.INITIALIZE_WEBCAM     = true;
        super.init();
        rrdrive = new SampleMecanumDrive(hardwareMap);
        autoPath = new AutoPath(hardwareMap,this);

        setStartPosisition(startPosition); //This is the enum startPosition above, which will be passed to TeleOp
        telemetry.addData("Start Position", startPosition.toString());
        telemetry.addData("State", "READY");
        currentState = BEGIN;
        iterationCounter = 1;

        lift.liftClaw.close();
        intake.setIntakeFlipperUp();
        if (!Calibration.DEBUG) {
            AutoTransitioner.transitionOnStop(this, "TeleOp");
        }
    }

    @Override
    public void init_loop() {
        super.init_loop();
        imu_loop();
    }

    private void imu_loop() {
        if (INITIALIZE_IMU && controlHub.initialized) {
            Orientation allAngles = controlHub.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double      globalAngle = allAngles.firstAngle;
            telemetry.addData("IMU angle:", " (%.2f)", globalAngle);
        }
    }

    protected State handleState(State state) {

        switch (state) {
            case BEGIN:
                return READ_WEBCAM;

            case READ_WEBCAM:
                //webcam.getVuforiaLocalization(); //Targets are too far away, use telephoto lens? TODO
                //webcam.getTensorFlowObjectDetection(); //Field of view is somehow too small, investigate later? TODO
                webcam.getTensorFlowObjectDetectionCurrentGame();
                return GET_AUTO_PATHS;
              
            case GET_AUTO_PATHS:
                String fieldObjectsLabel = "NONE";
                if (!webcam.tensorFlowDetectedObject.label.isEmpty()) { fieldObjectsLabel = webcam.tensorFlowDetectedObject.label.toUpperCase(); }
                String pathNameAsString = startPosition.toString()+"_"+ fieldObjectsLabel;
                if (pathNameAsString != null && !pathNameAsString.trim().isEmpty()) {
                    pathName = PathName.valueOf(pathNameAsString);
                }
                autoPath.getAutoPaths(pathName);

                if (!rrdrive.isBusy()) {
                    telemetry.addData("Starting driveToWobbleDrop autopath: ", time.seconds());
                    try {
                        rrdrive.setPoseEstimate(autoPath.startPose);
                        rrdrive.followTrajectory(autoPath.driveToWobbleDrop);
                    } catch (Exception e) {
                        telemetry.addData("Error running ", pathName.toString() + " driveToWobbleDrop");
                        telemetry.addData("Exception Message: ",e.toString());
                        return IDLE;
                    }
                    return DRIVE_TO_WOBBLE;
                }
                break;
            case DRIVE_TO_WOBBLE:
                //RoadRunner sets drop prep position on the way
                //RoadRunner sets drop position on the way

                return DROP_WOBBLE;
            case DROP_WOBBLE:
                //RoadRunner drops wobble on the way

                if (!rrdrive.isBusy()) {
                    try {
                        rrdrive.followTrajectory(autoPath.driveToShoot);
                    } catch (Exception e) {
                        telemetry.addData("Error running ", pathName.toString() + " driveToShoot");
                        telemetry.addData("Exception Message: ",e.toString());
                        return IDLE;
                    }
                    return DRIVE_TO_SHOOT;
                }
                break;
            case DRIVE_TO_SHOOT:
                //RoadRunner retracts the lift on the way
                //RoadRunner spins up the flywheel on the way

                if (!rrdrive.isBusy()) {
                    // Start the wait timer once we switch to the next state, this is so we can track how long we've been in the SHOOT state
                    shooterTriggerTimer.reset();
                    return SHOOT;
                }
                break;
            case SHOOT:
                //Shooter flywheel was spun up by roadrunner on the way
                if (!rrdrive.isBusy()) {
                    shooter.triggerRun();
                }

                // Check if the timer has exceeded the specified wait time
                // If so, move on to the DRIVE_TO_PARK state
                if (shooterTriggerTimer.seconds() >= shooterTriggerTime) {
                    //Shooter and trigger are stopped on the way by roadrunner
                    try {
                        rrdrive.followTrajectory(autoPath.driveToPark);
                    } catch (Exception e) {
                        telemetry.addData("Error running ", pathName.toString() + " driveToPark");
                        telemetry.addData("Exception Message: ",e.toString());
                        return IDLE;
                    }
                    parkTimer.reset();
                    return DRIVE_TO_PARK;
                }
                break;
            case DRIVE_TO_PARK:
                return PARK;
            case PARK:
                if (parkTimer.seconds() >= parkTime) {
                    lift.stop();
                    shooter.triggerStop();
                    return IDLE;
                } else {
                    lift.calibrate();
                    shooter.triggerBackOff();
                    intake.setIntakeFlipperDown();
                }
                break;
            default:
                break;
        }
        return state;
    }

    @Override
    public void loop() {
        double elapsed = time.seconds();

        telemetry.setAutoClear(false);
        telemetry.addData("timer: ", elapsed);
        iterationCounter += 1;
        if (currentState == IDLE) {
            telemetry.addData("State:", "" + currentState + " / " + iterationCounter);
        } else {
            telemetry.addData("State:", "pre " + currentState + " / " + iterationCounter);
            State new_state = handleState(currentState);
            telemetry.addData("State:", "post " + currentState + " / " + iterationCounter);
            if (new_state != currentState) {
                time.reset();
                currentState = new_state;
            }
        }

        //Update drive continuously in the background, regardless of state
        rrdrive.update();
        setPose(rrdrive.getPoseEstimate()); //This will be passed to TeleOp

        // Read current pose
        Pose2d poseEstimate = rrdrive.getPoseEstimate();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        if (Calibration.DEBUG) {
            util.sleep(500);
        }
    }

    @Override
    public void stop() {
        super.stop();
//        lift.setStorePosition();
//        if (webcam.initialized){ webcam.stop(); }
    }
    

}