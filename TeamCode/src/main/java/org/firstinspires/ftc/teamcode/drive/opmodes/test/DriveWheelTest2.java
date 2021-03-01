package org.firstinspires.ftc.teamcode.drive.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

@TeleOp(name="DriveWheelTest2", group = "TestCode")
@Disabled

public class DriveWheelTest2 extends RobotBase
{
    private ElapsedTime runtime = new ElapsedTime();
    private double maxSpeed;

    public DriveWheelTest2() {}
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        maxSpeed = 0.5;
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
        //set_drive_power(double RRWPower, double LRWPower, double RFWPower, double LFWPower)
        if (gamepad1.x == true){
            drive.leftFrontWheel.setPower(maxSpeed);
        } else {
            drive.leftFrontWheel.setPower(0);
        }
        if (gamepad1.y == true) {
            drive.rightFrontWheel.setPower(maxSpeed);
       } else {
            drive.rightFrontWheel.setPower(0);
        }
        if (gamepad1.a == true) {
            drive.leftRearWheel.setPower(maxSpeed);
        } else {
            drive.leftRearWheel.setPower(0);
        }
        if (gamepad1.b == true) {
            drive.rightRearWheel.setPower(maxSpeed);
        } else {
            drive.rightRearWheel.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
//    @Override
    public void stop() {
        drive.stop();
    }

}
