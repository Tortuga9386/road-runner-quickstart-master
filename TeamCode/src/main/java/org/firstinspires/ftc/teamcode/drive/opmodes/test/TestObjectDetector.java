package org.firstinspires.ftc.teamcode.drive.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.LocalGamepad;

@TeleOp(name="Test Object Detector", group="teleop")
@Disabled
public class TestObjectDetector extends RobotBase
{
    private ElapsedTime runtime = new ElapsedTime();
    public TestObjectDetector() {}
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized");
        webcam.getTensorFlowObjectDetection();
        telemetry.addData("Found Object", webcam.tensorFlowDetectedObject.label.toUpperCase());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

    }

}
