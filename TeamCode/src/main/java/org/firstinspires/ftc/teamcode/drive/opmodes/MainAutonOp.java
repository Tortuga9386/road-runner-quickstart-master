package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
        RED_RIGHT_QUAD,RED_RIGHT_SINGLE,RED_RIGHT_NONE
    }
    public PathName pathName;
    public AutoPath autoPath;
    public SampleMecanumDrive rrdrive;

    //Timers
    private final ElapsedTime time = new ElapsedTime();
    private long iterationCounter = 0;
    double dropWobbleTime = 2.0;
    ElapsedTime dropWobbleTimer = new ElapsedTime();
    double shooterTime = 2.0;
    ElapsedTime shooterTimer = new ElapsedTime();
    double shooterTriggerTime = 2.0;
    ElapsedTime shooterTriggerTimer = new ElapsedTime();

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

        telemetry.addData("Start Position", startPosition.toString());
        telemetry.addData("State", "READY");
        currentState = BEGIN;
        iterationCounter = 1;

        lift.liftClaw.close();
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
                    rrdrive.setPoseEstimate(autoPath.startPose);
                    rrdrive.followTrajectory(autoPath.driveToWobbleDrop);
                    return DRIVE_TO_WOBBLE;
                }
                break;
            case DRIVE_TO_WOBBLE:
                //Get ready to drop
                lift.setDropPosition();

                // Check if the drive class is still busy following the trajectory
                // If not, move onto the next state, DROP_WOBBLE
                if (!rrdrive.isBusy() && !lift.isBusy()) {
                    // Start the wait timer once we switch to the next state
                    // This is so we can track how long we've been in the DROP_WOBBLE state
                    dropWobbleTimer.reset();
                    return DROP_WOBBLE;
                }
                break;
            case DROP_WOBBLE:
                //Drop wobble
                lift.liftClaw.open();

                // Check if the timer has exceeded the specified wait time
                // If so, move on to the DRIVE_TO_SHOOT state
                if (dropWobbleTimer.seconds() >= dropWobbleTime) {
                    //lift.setStorePosition();
                    rrdrive.followTrajectory(autoPath.driveToShoot);
                    return DRIVE_TO_SHOOT;
                }
                break;
            case DRIVE_TO_SHOOT:
                if (!rrdrive.isBusy()) {
                    // Start the wait timer once we switch to the next state
                    // This is so we can track how long we've been in the SHOOT state
                    shooterTimer.reset();
                    return SHOOT;
                }
                break;
            case SHOOT:
                //shooter.run();
                if (shooterTimer.seconds() >= shooterTime) {
                    shooterTriggerTimer.reset();
                    //shooter.triggerrun();
                }
                // Check if the timer has exceeded the specified wait time
                // If so, move on to the DRIVE_TO_PARK state
                if (shooterTriggerTimer.seconds() >= shooterTriggerTime) {
                    //shooter.triggerstop();
                    //shooter.stop();
                    rrdrive.followTrajectory(autoPath.driveToPark);
                    return DRIVE_TO_PARK;
                }
                //return IDLE;
                break;
            case DRIVE_TO_PARK:
                return PARK;
            case PARK:
                return IDLE;
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