package pedroPathing.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "! BLUE GRAPEFRUIT WALNUT", group = "! SUPER Autonomous")
public class specimenAuto extends OpMode {
    //region Declare Hardware
    // Declare OpMode members for each of the 4 drive motors and 3 horizontal/vertical lift motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null; // Spongebob
    private DcMotor leftBackDrive = null; // Clarence
    private DcMotor rightFrontDrive = null; // Dumbledore
    private DcMotor rightBackDrive = null; // Gandalf
    private DcMotor verticalRight = null; // Aristotle
    private DcMotor verticalLeft = null; // Plato
    private DcMotor horizontalDrive = null; // Pythagoras

    // All 5 of the Intake Servos plugged into Expansion Hub 3
    private Servo intakeArm =  null; // Edward
    private Servo intakeClaw =  null; // Servo that opens and closes intake claw
    private Servo intakeRotate =  null; // Servo that rotates the claw left right
    private Servo intakeWrist =  null; // Servo that rotates the claw up down

    // All 3 of the Outtake Servos plugged into Control Hub
    private Servo deposClaw =  null; // Edward
    private Servo deposExtendo =  null; // Stuart
    private Servo deposArm =  null; // Felicia

    private AnalogInput depositEncoder1 = null;
    private AnalogInput depositEncoder2 = null;

    private AnalogInput wristEncoder1 = null;
    private AnalogInput wristEncoder2 = null;
    private AnalogInput armEncoder1 = null;
    private AnalogInput armEncoder2 = null;

    private DigitalChannel magLimVertical1 = null;
    private DigitalChannel magLimVertical2 = null;
    private DigitalChannel magLimHorizontal1 = null;
    private DigitalChannel magLimHorizontal2 = null;

    ContinuousServoController deposLeftController = null;
    ContinuousServoController deposRightController = null;
    ContinuousServoController wristServoController = null;
    ContinuousServoController intakeArmServoController = null;

    CRServo dummy = null;

    // Sensors
    private DistanceSensor backDistance = null;
    private Limelight3A limelight;
    private ColorSensor colorSensor = null;
    private Servo indicatorServo;

    boolean magHorOn = true; // Usually, "false" means pressed
    boolean magVertOn = true; // Usually, "false" means pressed
    boolean limitSwitchNotTriggered = true;
    //endregion

    //region Set States
    String robotState = "Transfer";
    String scoreState = "Sample";
    //endregion

    //region Declare Booleans
    // Servo Toggle Debounces
    Boolean intakeClawState = false; // true = open, false = close (i think)
    Boolean intakeRotateState = false; // false = transfer rotation
    Boolean deposClawState = true; // true = open, false = close
    Boolean deposArmState = false;

    Boolean intakeClawDebounce = false; // Claw Open
    Boolean intakeRotateDebounce = false; // Rotated in State 1
    Boolean deposClawDebounce = false;
    Boolean deposArmDebounce = false;

    Boolean horizontalDriveLockDebounce = false;
    Boolean horizontalDriveLockState = false;

    Boolean intakeState = true;
    Boolean intakeDebounce = false;

    Boolean grabbing = false;
    Boolean intakeWaitToReturn = false;
    double grabTimer = 0.0;

    Boolean autoIntakeMode = false;
    Boolean autoIntakeDebounce = false;

    Boolean intakeRotateOverride = false;

    double verticalZeroValue = 0;
    double verticalLiftValue = 0;

    double horizontalZeroValue = 0;
    double horizontalLiftValue = 0;

    double horizontalLiftTargetIN = 0;
    double xOffsetBlock = 0;
    MotorPIDController extendoController;
    MotorPIDController liftControllerRight;
    MotorPIDController liftControllerLeft;
    //endregion

    private double timeStamp = 0.0;
    private double colorvalue = 0.279;

    int chosenPose = 2;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int numberScored = 0;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 63, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(43, 70, Math.toRadians(0)); //13.5, 127.5

    private final Pose scorePose1 = new Pose(43, 72.5, Math.toRadians(0)); //13.5, 127.5
    private final Pose scorePose2 = new Pose(43, 70, Math.toRadians(0)); //13.5, 127.5
    private final Pose scorePose3 = new Pose(43, 67.5, Math.toRadians(0)); //13.5, 127.5

    private final Pose dropIntake = new Pose(18.407, 44.5, Math.toRadians(-108));

    private final Pose behindSub = new Pose(27.23, 70, Math.toRadians(-108)); //13.5, 127.5

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(25.7, 21, Math.toRadians(-33)); //x was 33.3 before

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(25.7, 21, Math.toRadians(-33));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(25.7, 21, Math.toRadians(-33)); // y was 129.55 before

    private final Pose dropPose1 = new Pose(24.7, 21, Math.toRadians(-136));

    private final Pose dropPose2 = new Pose(24.7, 21, Math.toRadians(-136));

    private final Pose dropPose3 = new Pose(24.7, 21, Math.toRadians(-136));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(13.230359520639148, 15.723035952063917, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(75.25091633466136, 139.0836653386454, Math.toRadians(90));
    private final Pose parkScoreControlPose = new Pose(75.25091633466136, 120.0836653386454, Math.toRadians(90));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park, scoreSub, backSub, dropGrab;
    private Path grabPickup1, grabPickup2, grabPickup3, dropPickup1, dropPickup2, dropPickup3;

    //private PathChain[] grabPaths = {grabPickup1, grabPickup2, grabPickup3};
    //private PathChain[] scorePaths = {dropPickup1, dropPickup2, dropPickup3};

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        dropGrab = new Path(new BezierLine(new Point(behindSub), new Point(dropIntake)));
        dropGrab.setLinearHeadingInterpolation(behindSub.getHeading(), dropIntake.getHeading());
        //dropGrab.setConstantHeadingInterpolation(scorePose.getHeading());


        backSub = new Path(new BezierLine(new Point(scorePose), new Point(behindSub)));
        //backSub.setLinearHeadingInterpolation(scorePose.getHeading(), dropIntake.getHeading());
        backSub.setConstantHeadingInterpolation(behindSub.getHeading());


        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = new Path(new BezierLine(new Point(dropIntake), new Point(pickup1Pose)));
        grabPickup1.setLinearHeadingInterpolation(dropIntake.getHeading(), pickup1Pose.getHeading());

        dropPickup1 = new Path(new BezierLine(new Point(pickup1Pose), new Point(dropPose1)));
        dropPickup1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), dropPose1.getHeading());

        grabPickup2 = new Path(new BezierLine(new Point(dropPose1), new Point(pickup2Pose)));
        grabPickup2.setLinearHeadingInterpolation(dropPose1.getHeading(), pickup2Pose.getHeading());

        dropPickup2 = new Path(new BezierLine(new Point(pickup2Pose), new Point(dropPose2)));
        dropPickup2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), dropPose2.getHeading());

        grabPickup3 = new Path(new BezierLine(new Point(dropPose2), new Point(pickup3Pose)));
        grabPickup3.setLinearHeadingInterpolation(dropPose2.getHeading(), pickup3Pose.getHeading());

        dropPickup3 = new Path(new BezierLine(new Point(pickup3Pose), new Point(dropPose3)));
        dropPickup3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), dropPose3.getHeading());


        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        scoreSub = new Path(new BezierCurve(new Point(parkPose), /* Control Point */ new Point(parkScoreControlPose), new Point(scorePose)));
        scoreSub.setLinearHeadingInterpolation(parkPose.getHeading(), scorePose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //region Initialize and Move to Score
                deposClawState = true;
                intakeClawState = false;
                intakeRotateState = false;
                autoIntakeMode = true;
                intakeState = false;

                follower.followPath(scorePreload, true);
                scoreState = "Specimen";
                setPathState(1);
                break;
                //endregion
            case 1:
                //region Bring Lift Up and Extendo Out as we move
                extendoController.setTargetPosition(1200, 1, "Ticks", horizontalLiftValue);

                liftControllerRight.setTargetPosition(1150, 1, "Ticks", verticalLiftValue);
                verticalLeft.setPower(verticalRight.getPower());

                if (!follower.isBusy() && Math.abs(verticalLiftValue - 1200) < 50) {
                    setPathState(2);
                    numberScored++;

                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
                //endregion
            case 2: // Initialize and move to score position for preload
                //region Begin Begin Clipping
                extendoController.setTargetPosition(1150, 1, "Ticks", horizontalLiftValue);

                deposClawState = false;

                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.33)) { // 0.3 before CHANGED
                    break;
                }

                liftControllerRight.setTargetPosition(1700, 1, "Ticks", verticalLiftValue);
                verticalLeft.setPower(verticalRight.getPower());

                if (Math.abs(verticalLiftValue - 1600) < 50) {
                    setPathState(3);
                    scoreState = "Sample";

                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                    break;
                }
                break;
                //endregion
            case 3:
                //region Calculate Required Extendo Position
                if (follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.25)) { // 0.3 before CHANGED
                    extendoController.setTargetPosition(1200, 1, "Ticks", horizontalLiftValue);
                    telemetry.addData("WAITING", "");

                    if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.15)) { // 0.3 before CHANGED
                        leftFrontDrive.setPower(-0.5);
                        rightFrontDrive.setPower(-0.5);
                        leftBackDrive.setPower(-0.5);
                        rightBackDrive.setPower(-0.5);
                    }
                    break;
                }

                liftControllerRight.setTargetPosition(25, 1, "Ticks", verticalLiftValue);
                verticalLeft.setPower(verticalRight.getPower());

                intakeState = false;
                autoIntakeMode = true;

                LLResult cameraResult = limelight.getLatestResult();
                double[] pythonOutputs = cameraResult.getPythonOutput();

                if (pythonOutputs[0] > 0.1) {
                    double blueXoffset = pythonOutputs[1];
                    double blueXoffsetIN = blueXoffset/120;
                    double blueYoffset = pythonOutputs[2];
                    double blueYoffsetIN = blueYoffset/120;

                    telemetry.addData("Horizontal Value", horizontalLiftValue);
                    telemetry.addData("Extendo Inches", extendoController.getCurrentPosition("Inches"));
                    telemetry.addData("Blue X Offset Inches", blueXoffsetIN);
                    telemetry.addData("Blue Y Offset Inches", blueYoffsetIN);

                    horizontalLiftTargetIN = extendoController.getCurrentPosition("Inches") - blueYoffsetIN - 1.5;
                    xOffsetBlock = blueXoffsetIN;

                    if (2 * blueXoffsetIN + horizontalLiftTargetIN > 11.2) {
                        extendoController.setTargetPosition(25, 0.60, "Ticks", horizontalLiftValue);
                        telemetry.addData("TOO FAR!", horizontalLiftValue);
                    } else {
                        setPathState(4);
                        break;
                    }
                } else {
                    extendoController.setTargetPosition(25, 0.60, "Ticks", horizontalLiftValue);

                    telemetry.addData("WE SEE NOTHING", horizontalLiftValue);
                }

                break;
                //endregion
            case 4:
                //region Move Extendo to Calculated Position
                if (follower.isBusy()) {
                    break;
                }

                liftControllerRight.setTargetPosition(25, 1, "Ticks", verticalLiftValue);
                verticalLeft.setPower(verticalRight.getPower());

                double posIN = extendoController.getCurrentPosition("Inches");

                telemetry.addData("Position Inches", posIN);
                telemetry.addData("Target Position", horizontalLiftTargetIN);

                extendoController.setTargetPosition(horizontalLiftTargetIN, 1, "Inches", horizontalLiftValue);

                if (Math.abs(posIN - horizontalLiftTargetIN) - 0 < 0.4) {
                    horizontalDrive.setPower(0);
                    telemetry.addData("Matching", "Positions");
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();

                    setPathState(5);
                    break;
                }

                break;
                //endregion
            case 5:
                //region Strafe to Block Position
                liftControllerRight.setTargetPosition(25, 1, "Ticks", verticalLiftValue);
                verticalLeft.setPower(verticalRight.getPower());

                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.33)) { // 0.3 before CHANGED
                    break;
                }

                intakeState = false;
                autoIntakeMode = true;

                cameraResult = limelight.getLatestResult();
                pythonOutputs = cameraResult.getPythonOutput();

                if (pythonOutputs[0] > 0.1) {
                    double blueXoffset = pythonOutputs[1];
                    double blueXoffsetIN = blueXoffset/120;
                    double blueYoffset = pythonOutputs[2];
                    double blueYoffsetIN = blueYoffset/120;

                    telemetry.addData("Horizontal Value", horizontalLiftValue);
                    telemetry.addData("Extendo Inches", extendoController.getCurrentPosition("Inches"));
                    telemetry.addData("Blue X Offset Inches", blueXoffsetIN);
                    telemetry.addData("Blue Y Offset Inches", blueYoffsetIN);

                    if (pythonOutputs[1] < 0) {
                        double lateral = -0.375;

                        double leftFrontPower = +lateral * 1.15;
                        double rightFrontPower = -lateral * 1.15;
                        double leftBackPower = -lateral;
                        double rightBackPower = +lateral;

                        leftFrontDrive.setPower(leftFrontPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        leftBackDrive.setPower(leftBackPower);
                        rightBackDrive.setPower(rightBackPower);
                    } else {
                        double lateral = 0.375;

                        double leftFrontPower = +lateral * 1.15;
                        double rightFrontPower = -lateral * 1.15;
                        double leftBackPower = -lateral;
                        double rightBackPower = +lateral;

                        leftFrontDrive.setPower(leftFrontPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        leftBackDrive.setPower(leftBackPower);
                        rightBackDrive.setPower(rightBackPower);
                    }

                    if (Math.abs(pythonOutputs[1]) < 55) {
                        timeStamp = opmodeTimer.getElapsedTimeSeconds();
                        setPathState(6);

                        double angle = pythonOutputs[3];
                        double calculatedPosition = 1 - (0.00337777 * angle);

                        grabTimer = runtime.seconds();
                        intakeRotateOverride = true;
                        intakeClawState = false;
                        grabbing = true;

                        intakeRotate.setPosition(calculatedPosition);
                        break;
                    }
                    break;
                } else {
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);

                    telemetry.addData("WE SEE NOTHING", horizontalLiftValue);
                }

                break;
                //endregion
            case 6:
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.85)) {
                    break;
                }

                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 1.25)) { // 0.3 before CHANGED
                    extendoController.setTargetPosition(25, 1, "Ticks", horizontalLiftValue);
                    break;
                }


                backSub = new Path(new BezierLine(new Point(follower.getPose()), new Point(behindSub)));
                backSub.setLinearHeadingInterpolation(follower.getPose().getHeading(), behindSub.getHeading());


                follower.followPath(backSub, true);
                timeStamp = opmodeTimer.getElapsedTimeSeconds();

                setPathState(7);
                break;
            case 7:
                if (follower.isBusy()) {
                    break;
                }
                follower.followPath(dropGrab, true);
                setPathState(8);
                break;
            case 8:
                if (follower.isBusy() || horizontalLiftValue < 1100) {
                    if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.75)) {
                        extendoController.setTargetPosition(1200, 1, "Ticks", horizontalLiftValue);
                    }
                    break;
                }
                timeStamp = opmodeTimer.getElapsedTimeSeconds();
                intakeClawState = false;
                setPathState(100);
                break;
            case 100:
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.5)) { // 0.3 before CHANGED
                    break;
                }

                switch (numberScored) {
                    case 1:
                        follower.followPath(grabPickup1, true);
                        break;
                    case 2:
                        follower.followPath(grabPickup2, true);
                        break;
                    case 3:
                        follower.followPath(grabPickup3, true);
                        break;
                    case 4:
                        setPathState(999);
                        break;
                }
                numberScored++;

                setPathState(101);
                break;
            case 101:
                if (follower.isBusy()) { // 0.3 before CHANGED
                    break;
                }

                switch (numberScored) {
                    case 2:
                        follower.followPath(dropPickup1, true);
                        break;
                    case 3:
                        follower.followPath(dropPickup2, true);
                        break;
                    case 4:
                        follower.followPath(dropPickup3, true);
                        break;
                }

                setPathState(102);
                break;
            case 102:
                if (follower.isBusy()) {
                    break;
                }

                setPathState(100);
                break;
            case 999:
                follower.turn(Math.toRadians(180), true);
                setPathState(1000);
                break;
            case 1000:
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        //region Color Lights :3
        indicatorServo.setPosition(colorvalue);

        colorvalue += 0.005;
        if (colorvalue > 0.72) {
            colorvalue = 0.28;
        }
        //endregion

        //region Magnetic Limit Switches
        magHorOn = !magLimHorizontal1.getState(); // Usually, "false" means pressed
        magVertOn = !magLimVertical1.getState(); // Usually, "false" means pressed

        if (magVertOn) {
            verticalZeroValue = (double) verticalRight.getCurrentPosition();
        }
        verticalLiftValue = (double) (verticalRight.getCurrentPosition() - verticalZeroValue);

        if (magHorOn) {
            horizontalZeroValue = (double) horizontalDrive.getCurrentPosition();
        }
        horizontalLiftValue = (double) (horizontalDrive.getCurrentPosition() - horizontalZeroValue);
        //endregion

        if (intakeState) {
            moveWristTo("Close", intakeWrist);
            if (Math.abs(wristServoController.getCurrentPositionInDegrees() - 65) <= 30) {
                if (intakeWaitToReturn) {
                    //intakeClawState = true;
                    moveArmTo("Wait", intakeArm);
                } else {
                    moveArmTo("Close", intakeArm);
                }
            }
        } else {
            //deposArmState = true;
            if (!grabbing) {
                moveWristTo("Open", intakeWrist);
            } else {
                moveWristTo("Grab", intakeWrist);
            }

            if ((Math.abs(wristServoController.getCurrentPositionInDegrees() - 15) <= 10) || (Math.abs(wristServoController.getCurrentPositionInDegrees() - 100) <= 10)) {
                if (!grabbing) {
                    //intakeClawState = false;
                    moveArmTo("Open", intakeArm);
                } else {
                    moveArmTo("Grab", intakeArm);
                }
            }
        }
        if (autoIntakeMode) {
            if (runtime.seconds() > grabTimer + 0.65 && grabbing) {
                grabbing = false;
                intakeRotateOverride = false;
            }

            if (runtime.seconds() > grabTimer + 0.35 && grabbing && runtime.seconds() < grabTimer + 0.6) {
                intakeClawState = true;
            }
        } else {
            if (runtime.seconds() > grabTimer + 0.3 && grabbing) {
                grabbing = false;
                intakeRotateOverride = false;
            }

            if (runtime.seconds() > grabTimer + 0.15 && grabbing && runtime.seconds() < grabTimer + 0.6) {
                intakeClawState = true;
            }
        }


        if (!intakeRotateOverride) {
            if (intakeRotateState) {
                intakeRotate.setPosition(0.38);
            } else {
                intakeRotate.setPosition(0.72);
            }
        }

        if (intakeClawState) { intakeClaw.setPosition(0); } else { intakeClaw.setPosition(1); }

        if (scoreState.equals("Sample")) {
            deposExtendo.setPosition(0.38);
            if (deposClawState && deposClaw.getPosition() != 1) { deposClaw.setPosition(1); } else if (!deposClawState && deposClaw.getPosition() != 0.45) { deposClaw.setPosition(0.45); }
        } else if (scoreState.equals("Specimen")) {
            deposClaw.setPosition(1);
            if (verticalLiftValue > 500) {
                if (deposClawState && deposExtendo.getPosition() != 0.38) {
                    deposExtendo.setPosition(0.38);
                } else if (!deposClawState && deposExtendo.getPosition() != 0.585) {
                    deposExtendo.setPosition(0.585);
                }
            } else {
                deposExtendo.setPosition(0.38);
            }
        }

        if (deposArmState) {
            switch (scoreState) {
                case "Sample":
                    moveDeposTo("Depos", deposArm);
                    break;
                case "Specimen":
                    moveDeposTo("Specimen", deposArm);
                    break;
            }
        } else {
            moveDeposTo("Transfer", deposArm);
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("rotation", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("scored count", numberScored);
        telemetry.addData("FOLLOWER BUSY", follower.isBusy());
        telemetry.addData("claw state", intakeClawState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("depos arm value!", deposLeftController.getCurrentPositionInDegrees());
        telemetry.addData("claw pos!", deposClaw.getPosition());
        telemetry.addData("vertical lift value", verticalLiftValue);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        //region Find Hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        // All of the lift thingamajigs
        verticalRight = hardwareMap.get(DcMotor.class, "verticalRight");
        verticalLeft = hardwareMap.get(DcMotor.class, "verticalLeft");
        horizontalDrive = hardwareMap.get(DcMotor.class, "horizontalDrive");


        // All 4 input servos
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw"); // Exp. Hub P4
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist"); // Exp. Hub P3
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P2
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P1

        // All 3 output servos
        deposClaw = hardwareMap.get(Servo.class, "deposClaw");
        deposExtendo = hardwareMap.get(Servo.class, "deposExtendo");
        deposArm = hardwareMap.get(Servo.class, "deposArm");

        // All 3 special servo encoders
        depositEncoder1 = hardwareMap.get(AnalogInput.class, "depositEncoder1");
        depositEncoder2 = hardwareMap.get(AnalogInput.class, "depositEncoder2");
        wristEncoder1 = hardwareMap.get(AnalogInput.class, "wristEncoder1");
        wristEncoder2 = hardwareMap.get(AnalogInput.class, "wristEncoder2");
        armEncoder1 = hardwareMap.get(AnalogInput.class, "armEncoder1");
        armEncoder2 = hardwareMap.get(AnalogInput.class, "armEncoder2");


        // All the digital magnetic limit switches
        magLimVertical1 = hardwareMap.get(DigitalChannel.class, "magLimVertical1");
        magLimVertical2 = hardwareMap.get(DigitalChannel.class, "magLimVertical2");
        magLimHorizontal1 = hardwareMap.get(DigitalChannel.class, "magLimHorizontal1");
        magLimHorizontal2 = hardwareMap.get(DigitalChannel.class, "magLimHorizontal2");

        // Get sensors here
        //backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        indicatorServo = hardwareMap.get(Servo.class, "indicator_servo");

        limelight.setPollRateHz(100);
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.reloadPipeline();
        //endregion

        //region Configure Hardware
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        verticalLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magLimVertical1.setMode(DigitalChannel.Mode.INPUT);
        magLimHorizontal1.setMode(DigitalChannel.Mode.INPUT);

        CRServo dummy = null;
        deposLeftController = new ContinuousServoController(dummy, depositEncoder1);
        deposRightController = new ContinuousServoController(dummy, depositEncoder1);
        wristServoController = new ContinuousServoController(dummy, wristEncoder1);
        intakeArmServoController = new ContinuousServoController(dummy, armEncoder1);
        //endregion
        horizontalDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoController = new MotorPIDController(horizontalDrive, 0.010, 0, 0.0003, 0.1, (double) (700 / 180), 537.6, 4.941);
        liftControllerRight = new MotorPIDController(verticalRight, 0.008, 0, 0.00055, 0.5, (double) (700 / 180), 384.5, 4.5);
        liftControllerLeft = new MotorPIDController(verticalRight, 0.008, 0, 0.00055, 0.5, (double) (700 / 180), 384.5, 4.5);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {
            chosenPose = 2;
        } else if (gamepad1.dpad_left) {
            chosenPose = 1;
        } else if (gamepad1.dpad_right) {
            chosenPose = 3;
        }

        telemetry.addData("Chosen Pose", chosenPose);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        // These lines for SUBMERSIBLE GRAB
        setPathState(0);
        //setPathState(999);

        switch (chosenPose) {
            case 2:
                scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose2)));
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose2.getHeading());
                break;
            case 1:
                scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose1)));
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
                break;
            case 3:
                scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose3)));
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose3.getHeading());
                break;
        }
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    //region Servo Helper Functions
    // Move intake wrist to "Open" or "Close"
    public void moveArmTo(String state, Servo arm) {
        switch (state) {
            case "Open": // Equal to grab position
                if (autoIntakeMode) {
                    arm.setPosition(0.7);
                } else {
                    arm.setPosition(0.575);
                }
                break;
            case "Close": // Equal to transfer position
                arm.setPosition(0.9); // 0.9 before
                break;
            case "Grab": // Equal to grab position
                arm.setPosition(0.475);
                break;
            case "Wait": // Wait to return position
                arm.setPosition(0.75);
                break;
        }
    }

    // Move intake wrist to "Open" or "Close"
    public void moveWristTo(String state, Servo wrist) {
        switch (state) {
            case "Open": // Equal to grab position
                if (autoIntakeMode) {
                    wrist.setPosition(1);
                } else {
                    wrist.setPosition(0.735);
                }
                break;
            case "Close": // Equal to transfer position
                wrist.setPosition(0.15);
                break;
            case "Grab":
                wrist.setPosition(0.705);
                break;
        }
    }


    public void moveDeposTo(String state, Servo arm) {
        switch (state) {
            case "Transfer": // Equal to grab position
                arm.setPosition(0.3);
                break;
            case "Depos": // Equal to transfer position
                arm.setPosition(0.725);
                break;
            case "Specimen":
                arm.setPosition(0.78);
                break;
        }
    }
    //endregion

    //region State Finding Functions
    // Check if horizontal slides are all the way in
    public Boolean extendoClosed() { return (horizontalLiftValue < 25); }
    // Check if lift is all the way down
    public Boolean liftDown(DigitalChannel limitSwitch) { return (!limitSwitch.getState()); }
    // Check if wrist and arm are back and claw is rotated in transfer position
    public Boolean intakeInTransferPosition(ContinuousServoController controllerWrist) { return (Math.abs(controllerWrist.getCurrentPositionInDegrees() - 63.70) < 5); }
    // Check if the depos arm is down
    public Boolean deposArmDown(ContinuousServoController controllerDepos) { return (Math.abs(controllerDepos.getCurrentPositionInDegrees() - 32) < 3); }
    // Check if the depos claw is closed
    public Boolean deposClawClosed() { return deposClawState; }

    // Get the state of the robot based on other values; can be overridden by certain controls
    public String getRobotState(ContinuousServoController controllerWrist, ContinuousServoController controllerDepos, DigitalChannel limitSwitch) {
        if (extendoClosed() && liftDown(limitSwitch) && deposArmDown(controllerDepos) && intakeInTransferPosition(controllerWrist) && !deposClawClosed()) {
            // If everything retracted and depos claw open, basically starting position
            return "Transfer Ready";
        } else if (extendoClosed() && liftDown(limitSwitch) && deposArmDown(controllerDepos) && intakeInTransferPosition(controllerWrist) && deposClawClosed()) {
            // If everything retracted and depos claw closed, basically starting position
            return "Transfer Complete";
        } else if (!extendoClosed() || !intakeInTransferPosition(controllerWrist)) {
            // If extendo not in and arm/wrist not retracted
            return "Grab";
        } else if (!liftDown(limitSwitch) || !deposArmDown(controllerDepos)) {
            // If lift not down
            return "Depos";
        }
        return "Unknown";
    }
    //endregion
}

