package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.subsystems.*;

/**
 * This is NOT an opmode.
 *
 * This class initiates all the specific hardware for a robot.
 * The various subsystems classes are located in the subsystem folder.
 * This code is used by all the other opmodes.
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 */
@Disabled
public class RobotBase extends OpMode
{
    //Define constants that should NOT be adjusted by manual calibration
    public static double  COUNTS_PER_INCH       = 1892.3686435854;
    protected boolean INITIALIZE_WEBCAM         = false;
    protected boolean INITIALIZE_IMU            = false;
    protected boolean INITIALIZE_LSA            = false;


    //Make subsystems available to all class extensions
    public MenuController menu_controller;
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Lift lift;
    public Lights lights;
    public Webcam webcam;
    public ControlHub controlHub;
    public LocalizationSA localizationSA;
    public Util util;

    /* Constructor */
    public RobotBase(){ }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init() {
        //Read calibration constants
        //new Calibration().readFromFile();

        //Initialize subsystems
        menu_controller = new MenuController(new Calibration());
        drive = new Drive(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        shooter = new Shooter(hardwareMap, this);
        lift = new Lift(hardwareMap, this);
        util = new Util();

        /*
            Lights are only used by webcam, initialize them there only?
         */
        lights = new Lights(hardwareMap, this);
        lights.setPattern("GREEN");

        //Enable IMU for angles
        if (INITIALIZE_WEBCAM) {
            webcam = new Webcam(hardwareMap, this);
        }

        //Enable IMU for angles
        if (INITIALIZE_IMU) {
            controlHub = new ControlHub(hardwareMap, this);
        }

        //Enable Stand Alone Localization (NOT ROADRUNNER)
        if (INITIALIZE_LSA) {
            localizationSA = new LocalizationSA(hardwareMap, this);
        }

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        //Do nothing, use these classes in the opModes
    }

    /*
     * Code to run when the op mode is first disabled goes here
     */
    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        shooter.stop();
        lift.stop();
        lights.stop();

        if (webcam != null && webcam.initialized) {
            webcam.stop();
        }

        if (controlHub != null && controlHub.initialized) {
            controlHub.stop();
        }

        if (localizationSA != null && localizationSA.initialized) {
            localizationSA.stop();
        }
    }
    
 }
