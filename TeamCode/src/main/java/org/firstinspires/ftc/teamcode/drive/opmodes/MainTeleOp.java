package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.subsystems.LocalGamepad;

@TeleOp(name="TeleOp", group="teleop")
public class MainTeleOp extends RobotBase
{
    private ElapsedTime runtime = new ElapsedTime();
    private LocalGamepad localGamepad1, localGamepad2;
    private double turtleToggleUpdate = 0.0;

    public MainTeleOp() {}
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        localGamepad1 = new LocalGamepad(gamepad1);
        localGamepad2 = new LocalGamepad(gamepad2);

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
        localGamepad1.update();
        localGamepad2.update();

        telemetry.addData("Status", "Run Time:" + runtime.toString());

        drive_loop();
        intake_loop();
        shooter_loop();
        lift_loop();
        light_loop();
        telemetry_loop();
    }

    protected void drive_loop() {
        
        if (turtleToggleUpdate + Calibration.MIN_TOGGLE_TIME<=runtime.seconds()) {
            if (gamepad1.a && !drive.turtleMode){
                drive.turtleMode  = true;
                turtleToggleUpdate = runtime.seconds();
            } else if (gamepad1.a && drive.turtleMode) {
                drive.turtleMode  = false;
                turtleToggleUpdate = runtime.seconds();
            }
        }
        telemetry.addData("Turtle Mode On", drive.turtleMode);
        
        //Use localGamepad1.XasCircle and localGamepad1.YasCircle to prevent skewing in corners of joystick square
        drive.basicDrive(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x);
    }


    protected void intake_loop() {
        //Activate intake based on left trigger
        if (gamepad1.left_trigger == 1){
            intake.run();
            telemetry.addData("Intake Power", "ON");
        }else if(gamepad1.left_bumper){
            intake.reverse();
        }
        else {
            intake.stop();
            telemetry.addData("Intake Power", "OFF");
        }

    }

    protected void shooter_loop() {
        //Activate flywheel and shooter servo based on button combos
        if (gamepad1.right_trigger > 0.0){
            shooter.run();
        } else {
            //Turn off shooter flywheel if right trigger is not held down
            shooter.stop();
        }
        if (gamepad1.right_bumper) {
            shooter.triggerrun();
            telemetry.addData("Shooter Servo Power", true);
        } else {
            shooter.triggerstop();
        }
    
    }
    
    protected void lift_loop() {
        //Run lift and lift claw based on button combos
        if (gamepad1.dpad_up){
            lift.setDropPosition();
            telemetry.addData("lift setDropPosition", true);
        } else if (gamepad1.dpad_down){
            lift.setStorePosition();
            telemetry.addData("lift setStorePosition", true);
        } else {
            lift.stop();
            telemetry.addData("lift stop", true);
        }
        telemetry.addData("Lift current Position", lift.liftMotor.getCurrentPosition());
        telemetry.addData("Lift store sensor", lift.liftStopStore.getState());

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

    protected void telemetry_loop() {

        if (INITIALIZE_LSA && localizationSA.initialized) {
            // Show the elapsed run time, magnitude, direction and wheel power.
            telemetry.addData("X Position", util.getCountsAsInches(localizationSA.globalPositionUpdate.returnXCoordinate()));
            telemetry.addData("Y Position", util.getCountsAsInches(localizationSA.globalPositionUpdate.returnYCoordinate()));
            telemetry.addData("Orientation (Radians)", util.roundTwo(localizationSA.globalPositionUpdate.returnOrientationScaledRadians()));
        }

        if (INITIALIZE_IMU && controlHub.initialized) {
            Orientation allAngles = controlHub.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double      globalAngle = allAngles.firstAngle;
            telemetry.addData("IMU angle:", " (%.2f)", globalAngle);
        }
    }

}
