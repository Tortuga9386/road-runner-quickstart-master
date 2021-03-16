package org.firstinspires.ftc.teamcode.drive.opmodes;

import java.lang.Math;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.subsystems.LocalGamepad;
import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.*;

@TeleOp(name="TeleOp", group="teleop")
public class MainTeleOp extends RobotBase
{
    private   ElapsedTime   runtime = new ElapsedTime();
    private   LocalGamepad  localGamepad1, localGamepad2;
    private   double        turtleToggleUpdate = 0.0;
    protected boolean       INITIALIZE_IMU = true;
    protected double        globalAngle = 0.0;

    //Keep track of where we are, this is necessary for the alignToShoot function
    StandardTrackingWheelLocalizer rrLocalizer;

    public MainTeleOp() {}
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        localGamepad1 = new LocalGamepad(gamepad1);
        localGamepad2 = new LocalGamepad(gamepad2);
        rrLocalizer   = new StandardTrackingWheelLocalizer(hardwareMap);

        //Set your initial pose from position stored from AutoOp
        rrLocalizer.setPoseEstimate(getPose());
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time:" + runtime.toString());
        localGamepad1.update();
        localGamepad2.update();

        drive_loop();
        intake_loop();
        shooter_loop();
        lift_loop();
        light_loop();
        imu_loop();
        telemetry_loop();
    }

    protected void drive_loop() {

        if (turtleToggleUpdate + Calibration.MIN_TOGGLE_TIME<=runtime.seconds()) {
            if (gamepad1.a && !this.turtleMode){
                updateTurtleMode(true);
                turtleToggleUpdate = runtime.seconds();
            } else if (gamepad1.a && this.turtleMode) {
                updateTurtleMode(false);
                turtleToggleUpdate = runtime.seconds();
            }
        }
        telemetry.addData("Turtle Mode", this.turtleMode);
        
        // Make sure to call myLocalizer.update() on *every* loop
        // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
        rrLocalizer.update();
        setPose(rrLocalizer.getPoseEstimate()); //Store pose for future interactions
        if (gamepad1.b && gamepad1.y) {
            drive.alignToShoot();
        } //move this to an case below with drive modes to block drivers from interrupting?

        //Use localGamepad1.XasCircle and localGamepad1.YasCircle to prevent skewing in corners of joystick square
        telemetry.addData("Drive Mode", driveMode.toString());
        switch (driveMode) {
            case ARC_DRIVE_ROBOT:
                driveArcRobotics.driveRobotCentric(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x);
                break;
            case ARC_DRIVE_FIELD:
                driveArcRobotics.driveFieldCentric(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x, globalAngle);
                break;
            case ORIGINAL_CRISPY:
            default:
                drive.driveRobotCentric(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x);
        }
        if (false) { //button combination to rotate through drive modes, really shouldnt be doing this in a match!
            //driveMode = getNextDriveMode(driveMode);
        }
    }
    public void updateTurtleMode(boolean newTurtleMode) {
        this.turtleMode = newTurtleMode;
        if (drive != null)    { drive.turtleMode  = newTurtleMode; }
        if (driveArcRobotics != null) { driveArcRobotics.turtleMode = newTurtleMode; }
    }
    public DriveMode getNextDriveMode(DriveMode oldDriveMode) {
        int index = oldDriveMode.ordinal();
        int nextIndex = index + 1;
        DriveMode[] driveModes = DriveMode.values();
        nextIndex %= driveModes.length;
        return driveModes[nextIndex];
    }

    protected void intake_loop() {
        //Activate intake based on left trigger
        if (gamepad1.left_trigger == 1) {
            intake.run();
            telemetry.addData("Intake Power", "ON");
        } else if(gamepad1.left_bumper) {
            intake.reverse();
            telemetry.addData("Intake Power", "REVERSE");
        } else {
            intake.stop();
            telemetry.addData("Intake Power", "OFF");
        }

        if (gamepad1.b) {
            intake.setIntakeFlipperDown();
            telemetry.addData("Intake Flipper", "UP");
        }
        if (gamepad1.y) {
            intake.setIntakeFlipperUp();
            telemetry.addData("Intake Flipper", "DOWN");
        }
//        if (gamepad1.b == true) {
//            intake.setIntakeFlipperUp();
//            telemetry.addData("Intake Flipper", "UP");
//        } else if(gamepad1.b == false) {
//            intake.setIntakeFlipperDown();
//            telemetry.addData("Intake Flipper", "DOWN");
//        }

    }

    protected void shooter_loop() {
        //Activate flywheel and shooter servo based on button combos
        if (gamepad1.right_trigger == 1){
            shooter.run();
            telemetry.addData("Shooter Power", "ON");
        } else {
            //Turn off shooter flywheel if right trigger is not held down
            shooter.stop();
            telemetry.addData("Shooter Power", "OFF");
        }
        if (gamepad1.right_bumper) {
            shooter.triggerRun();
            telemetry.addData("Shooter Servo Power", true);
        } else {
            shooter.triggerStop();
        }
    
    }
    
    protected void lift_loop() {
        //Run lift and lift claw based on button combos
        if (gamepad1.dpad_up){
            lift.setDropPosition();
            telemetry.addData("Lift Action", "lift.setDropPosition");
        } else if (gamepad1.dpad_down){
            lift.setStorePosition();
            telemetry.addData("Lift Action", "lift.setStorePosition");
        } else if (gamepad1.x) {
            lift.calibrate(); //Emergency lift reset!
            telemetry.addData("Lift Action", "lift.calibrate");
        } else {
            lift.stop();
            telemetry.addData("Lift Action", "lift.stop");
        }
        telemetry.addData("Lift Position", lift.liftMotor.getCurrentPosition());
        telemetry.addData("Lift Sensor", lift.liftStopStoreState());

        if (gamepad1.dpad_left) {
            lift.liftClaw.open();
            telemetry.addData("Lift Servo", "OPEN");
        } else if (gamepad1.dpad_right) {
            lift.liftClaw.close();
            telemetry.addData("Lift Servo", "CLOSED");
        }

    }

    protected void light_loop() {
        String lightPattern;
        
        if (gamepad1.x){
            lightPattern = "BLUE";
        } else if (gamepad1.a) {
            lightPattern = "GREEN";
        } else if (gamepad1.y) {
            lightPattern = "ORANGE";
        } else if (gamepad1.b) {
            lightPattern = "RED";
        } else {
            lightPattern = "BLACK";
        }

        lights.setPattern(lightPattern);
    }

    protected void imu_loop() {
        if (INITIALIZE_IMU && controlHub!=null && controlHub.initialized) {
            Orientation allAngles = controlHub.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            globalAngle = allAngles.firstAngle;
        }
    }

    protected void telemetry_loop() {

        if (rrLocalizer!= null) {
            Pose2d currentPose = rrLocalizer.getPoseEstimate();
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading radians", currentPose.getHeading());
            telemetry.addData("heading degrees", Math.toDegrees(currentPose.getHeading()));
        }

        if (INITIALIZE_LSA && localizationSA!= null && localizationSA.initialized) {
            // Show the elapsed run time, magnitude, direction and wheel power.
            telemetry.addData("X Position", util.getCountsAsInches(localizationSA.globalPositionUpdate.returnXCoordinate()));
            telemetry.addData("Y Position", util.getCountsAsInches(localizationSA.globalPositionUpdate.returnYCoordinate()));
            telemetry.addData("Orientation (Radians)", util.roundTwo(localizationSA.globalPositionUpdate.returnOrientationScaledRadians()));
        }

        if (INITIALIZE_IMU && controlHub!=null && controlHub.initialized) {
            telemetry.addData("IMU angle:", " (%.2f)", globalAngle);
        }
    }

}
