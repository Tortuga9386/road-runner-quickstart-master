package org.firstinspires.ftc.teamcode.drive.localizationstandalone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
@Disabled

public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;
    private int         verticalLeftEncoderCount = 0;
    private int         verticalRightEncoderCount = 0;
    private int         horizontalEncoderCount = 0;
    private double      verticalLeftEncoderDist = 0.0;
    private double      verticalRightEncoderDist = 0.0;
    private double      horizontalEncoderDist = 0.0;    

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 1892.3686435854;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    //String verticalLeftEncoderName = rfName, verticalRightEncoderName = rbName, horizontalEncoderName = lfName;
    //String verticalLeftEncoderName = "RightFrontWheel", verticalRightEncoderName = "LeftFrontWheel", horizontalEncoderName = "LeftRearWheel";
    String verticalLeftEncoderName = "RightFrontWheel", verticalRightEncoderName = "RightRearWheel", horizontalEncoderName = "LeftFrontWheel";

    @Override
    public void runOpMode() throws InterruptedException {

        //Assign the hardware map to the odometry wheels
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            verticalRightEncoderCount   = verticalRight.getCurrentPosition();
            verticalLeftEncoderCount    = verticalLeft.getCurrentPosition();
            horizontalEncoderCount      = horizontal.getCurrentPosition();
            verticalRightEncoderDist    = verticalRightEncoderCount / COUNTS_PER_INCH;
            verticalLeftEncoderDist     = verticalLeftEncoderCount / COUNTS_PER_INCH;
            horizontalEncoderDist       = horizontalEncoderCount / COUNTS_PER_INCH;

            // Show the elapsed run time, magnitude, direction and wheel power.
            telemetry.addData("Right VPod Encoder:", " (%d)", verticalRightEncoderCount);
            telemetry.addData("Left VPod Encoder:", " (%d)", verticalLeftEncoderCount);
            telemetry.addData("Rear HPod Encoder:", " (%d)", horizontalEncoderCount);
            telemetry.addData("Right VPod Encoder:", " (%.2f)", verticalRightEncoderDist);
            telemetry.addData("Left VPod Encoder:", " (%.2f)", verticalLeftEncoderDist);
            telemetry.addData("Rear HPod Encoder:", " (%.2f)", horizontalEncoderDist);
            
            
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}
