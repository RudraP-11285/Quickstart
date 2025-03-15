package pedroPathing.teamcode;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name = "! BLUE PINEAPPLE COCONUT", group = "! SUPER Autonomous")
public class sampleAuto extends OpMode {
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
    MotorPIDController extendoController;
    //endregion

    private double timeStamp = 0.0;
    private double colorvalue = 0.279;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int numberScored = 0;
    int yOffset = 0;
    double xOffsetBlock = 0;
    double horizontalLiftTargetIN = 0;

    boolean deposWait = false;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 104, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    public static final Pose scorePose = new Pose(13.75, 130.25, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    public static Pose pickup1Pose = new Pose(26.35, 122.9, Math.toRadians(0)); //x was 26.15 before

    /** Middle (Second) Sample from the Spike Mark */
    public static Pose pickup2Pose = new Pose(25.8, 132.9, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    //public static Pose pickup3Pose = new Pose(27,  128.5, Math.toRadians(135)); // y was 129.55 before
    private final Pose pickup3Pose = new Pose(32.42, 144 - 16.031291611185091, Math.toRadians(50)); // x was 32.22 before


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(66.28286852589642, 102.91235059760957, Math.toRadians(-85));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(50.812250332889484, 121.75765645805592, Math.toRadians(90));
    private final Pose parkScoreControlPose = new Pose(50.812250332889484, 121.75765645805592, Math.toRadians(90));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    //private Path park, scoreSub;
    private PathChain park, scoreSub, scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    private PathChain[] grabPaths = {grabPickup1, grabPickup2, grabPickup3};
    private PathChain[] scorePaths = {scorePickup1, scorePickup2, scorePickup3};

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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup1.getPath(0).setZeroPowerAccelerationMultiplier(2.75);

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        grabPickup2.getPath(0).setZeroPowerAccelerationMultiplier(2.75);


        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        grabPickup3.getPath(0).setZeroPowerAccelerationMultiplier(2.75);


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose))).setTangentHeadingInterpolation().setReversed(false).build();

        scoreSub = follower.pathBuilder().addPath(new BezierCurve(new Point(parkPose), /* Control Point */ new Point(parkScoreControlPose), new Point(scorePose))).setTangentHeadingInterpolation().setReversed(true).build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initialize and move to score position for preload
                deposExtendo.setPosition(0.41);

                deposClawState = true;
                intakeClawState = false;
                intakeRotateState = false;
                deposWait = false;

                switch (numberScored) {
                    case 0:
                        follower.followPath(scorePreload, true);
                        break;
                    default:
                        break;
                }

                timeStamp = opmodeTimer.getElapsedTimeSeconds();
                setPathState(1);
                break;
            case 1: // Bring the lift up as we move
                if (verticalLiftValue < 950) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                }

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.05)) { // 0.3 before CHANGED
                    deposWait = true;
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && (!(verticalLiftValue < 900))) {
                    /* Score Preload */
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(2);

                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 2: // Bring up the arm, wait, and drop
                deposWait = false;
                deposArmState = true;

                if (verticalLiftValue < 1000) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                } else {
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                }

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.12)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = false;
                    setPathState(3);
                    intakeState = false;
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 3: // Wait for drop, start moving to grab
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) { // 0.3 before CHANGED
                    break;
                }
                deposClawState = true;
                numberScored++;
                timeStamp = opmodeTimer.getElapsedTimeSeconds();

                switch (numberScored) {
                    case 1:
                        follower.followPath(grabPickup1, true);
                        setPathState(4);
                        break;
                    case 2:
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                        break;
                    case 3:
                        follower.followPath(grabPickup3, true);
                        setPathState(4);
                        break;
                    case 4:
                        follower.followPath(park, false);
                        deposClawState = true;
                        intakeClawState = false;
                        intakeRotateState = false;
                        setPathState(101);
                        break;
                }
                break;
            case 4: // Go to the next grab path
                //follower.followPath(grabPaths[numberScored - 1], true);
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) { // 0.3 before CHANGED
                    break;
                }

                deposArmState = false;

                setPathState(5);
                break;
            case 5: // Bring stuff down, grab when in position
                if (verticalLiftValue > 5) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }

                intakeState = false;
                if (numberScored == 3) {
                    intakeRotateState = true;
                    extendoController.setTargetPosition(250, 1, "Ticks", horizontalLiftValue);
                } else {
                    extendoController.setTargetPosition(235, 1, "Ticks", horizontalLiftValue);
                }


                if (!follower.isBusy() && Math.abs(intakeArmServoController.getCurrentPositionInDegrees() - 34) < 4) {
                    if (numberScored == 3 && Math.abs(horizontalLiftValue - 250) < 10) {
                        grabbing = true;
                        grabTimer = runtime.seconds();
                        setPathState(6);
                        timeStamp = opmodeTimer.getElapsedTimeSeconds();

                        double angle = 130;
                        double calculatedPosition = 1 - (0.00337777 * angle);

                        intakeRotateOverride = true;
                        intakeClawState = false;

                        intakeRotate.setPosition(calculatedPosition);

                        verticalLeft.setPower(0);
                        verticalRight.setPower(0);
                    } else if (Math.abs(horizontalLiftValue - 235) < 10) {
                        grabbing = true;
                        grabTimer = runtime.seconds();
                        setPathState(6);
                        timeStamp = opmodeTimer.getElapsedTimeSeconds();

                        verticalLeft.setPower(0);
                        verticalRight.setPower(0);
                    }
                }
                break;
            case 6: // Wait for grab to complete
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { // 0.5 before CHANGED //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(7);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 7:
                switch (numberScored) {
                    case 1:
                        follower.followPath(scorePickup1, true);
                        break;
                    case 2:
                        follower.followPath(scorePickup2, true);
                        break;
                    case 3:
                        follower.followPath(scorePickup3, true);
                        break;
                }

                setPathState(8);
                break;
            case 8: // Go to transfer position
                if (Math.abs(horizontalLiftValue - 100) > 7 && !intakeInTransferPosition(wristServoController)) {
                    extendoController.setTargetPosition(100, 1, "Ticks", horizontalLiftValue);
                }


                intakeRotateState = false;
                intakeClawState = true;
                deposArmState = false;
                deposClawState = false;


                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0)) {
                    break;
                }


                intakeState = true;


                if (verticalLiftValue > 3) {
                    verticalRight.setPower(-1);
                    verticalLeft.setPower(1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);

                    if (intakeInTransferPosition(wristServoController)) {
                        if (!magHorOn) { // 10 BEFORE CHANGE
                            horizontalDrive.setPower(-1);
                        } else {
                            horizontalDrive.setPower(0);

                            setPathState(9);
                            timeStamp = opmodeTimer.getElapsedTimeSeconds();
                        }
                    }
                }
                break;
            case 9:
                // transfer
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.1)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = true;
                    setPathState(10);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 10:
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.3)) { // 0.3 BEFORE changed //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    intakeClawState = false;
                    if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.2)) {
                        setPathState(0);
                        timeStamp = opmodeTimer.getElapsedTimeSeconds();
                    }
                }
                break;
            case 101:
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) { // 0.3 before CHANGED
                    break;
                }

                deposArmState = false;

                intakeState = false;
                autoIntakeMode = true;

                if (verticalLiftValue >= 15) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }

                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.7)) { // 0.3 before CHANGED
                    break;
                }

                if (horizontalLiftValue < 380) {
                    horizontalDrive.setPower(0.75);
                } else {
                    horizontalDrive.setPower(0);
                }

                if (!follower.isBusy() && !(horizontalLiftValue < 380) && !(verticalLiftValue >= 15)) {
                    setPathState(102);
                }
                break;
            case 102:
                intakeState = false;
                autoIntakeMode = true;
                deposArmState = false;


                if (follower.isBusy()) { // 0.3 before CHANGED
                    extendoController.setTargetPosition(380 - yOffset, 1, "Ticks", horizontalLiftValue);
                    telemetry.addData("WAITING", "");
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                    break;
                }


                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.5)) {
                    break;
                }


                if (verticalLiftValue >= 15) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }


                LLResult cameraResult = limelight.getLatestResult();
                double[] pythonOutputs = cameraResult.getPythonOutput();

                if (pythonOutputs[7] > 0.1) {
                    double yellowXoffset = pythonOutputs[8];
                    double yellowXoffsetIN = yellowXoffset/120;
                    double yellowYoffset = pythonOutputs[9];
                    double yellowYoffsetIN = yellowYoffset/120;

                    telemetry.addData("Horizontal Value", horizontalLiftValue);
                    telemetry.addData("Extendo Inches", extendoController.getCurrentPosition("Inches"));
                    telemetry.addData("Yellow X Offset Inches", yellowXoffsetIN);
                    telemetry.addData("Yellow Y Offset Inches", yellowYoffsetIN);

                    horizontalLiftTargetIN = extendoController.getCurrentPosition("Inches") - yellowYoffsetIN - 1.55;
                    xOffsetBlock = yellowYoffsetIN;

                    if (2 * yellowXoffsetIN + horizontalLiftTargetIN > 10) {
                        extendoController.setTargetPosition(25 / 2.642611684, 0.35, "Ticks", horizontalLiftValue);
                        telemetry.addData("TOO FAR!", horizontalLiftValue);
                    } else {
                        extendoController.setPIDF(0.013, 0, 0.000475, 0.1);
                        setPathState(103);
                        break;
                    }
                } else {
                    extendoController.setTargetPosition(25 / 2.642611684, 0.225, "Ticks", horizontalLiftValue);

                    telemetry.addData("WE SEE NOTHING", horizontalLiftValue);
                }

                break;
            case 103:
                if (verticalLiftValue >= 15) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }

                double posIN = extendoController.getCurrentPosition("Inches");

                telemetry.addData("Position Inches", posIN);
                telemetry.addData("Target Position", horizontalLiftTargetIN);

                extendoController.setTargetPosition(horizontalLiftTargetIN, 1, "Inches", horizontalLiftValue);

                if (Math.abs(posIN - horizontalLiftTargetIN) - 0 < 0.4) {
                    horizontalDrive.setPower(0);
                    telemetry.addData("Matching", "Positions");
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();

                    setPathState(104);
                    break;
                }

                break;
            case 104:
                //region Strafe to Block Position and Grab
                if (verticalLiftValue >= 15) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }

                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.33)) { // 0.3 before CHANGED
                    break;
                }

                intakeState = false;
                autoIntakeMode = true;

                cameraResult = limelight.getLatestResult();
                pythonOutputs = cameraResult.getPythonOutput();

                if (pythonOutputs[7] > 0.1) {
                    double yellowXoffset = pythonOutputs[8];
                    double yellowXoffsetIN = yellowXoffset/120;
                    double yellowYoffset = pythonOutputs[9];
                    double yellowYoffsetIN = yellowYoffset/120;

                    telemetry.addData("Horizontal Value", horizontalLiftValue);
                    telemetry.addData("Extendo Inches", extendoController.getCurrentPosition("Inches"));
                    telemetry.addData("Yellow X Offset Inches", yellowXoffsetIN);
                    telemetry.addData("Yellow Y Offset Inches", yellowYoffsetIN);

                    if (pythonOutputs[8] < 0) {
                        double lateral = -0.245;

                        double leftFrontPower = +lateral * 1.15;
                        double rightFrontPower = -lateral * 1.15;
                        double leftBackPower = -lateral;
                        double rightBackPower = +lateral;

                        leftFrontDrive.setPower(leftFrontPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        leftBackDrive.setPower(leftBackPower);
                        rightBackDrive.setPower(rightBackPower);
                    } else {
                        double lateral = 0.245;

                        double leftFrontPower = +lateral * 1.15;
                        double rightFrontPower = -lateral * 1.15;
                        double leftBackPower = -lateral;
                        double rightBackPower = +lateral;

                        leftFrontDrive.setPower(leftFrontPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        leftBackDrive.setPower(leftBackPower);
                        rightBackDrive.setPower(rightBackPower);
                    }

                    if (Math.abs(pythonOutputs[8]) < 55) {
                        timeStamp = opmodeTimer.getElapsedTimeSeconds();
                        setPathState(105);

                        double angle = pythonOutputs[10];
                        double calculatedPosition = 1 - (0.00337777 * angle);

                        leftFrontDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);

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
                    setPathState(102);
                }

                break;
            //endregion
            case 105: // Go to transfer position
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.85)) {
                    break;
                }

                if (Math.abs(horizontalLiftValue - 100) > 7 && !intakeInTransferPosition(wristServoController)) {
                    extendoController.setTargetPosition(100, 1, "Ticks", horizontalLiftValue);
                }


                intakeRotateState = false;
                intakeClawState = true;
                deposArmState = false;
                deposClawState = false;


                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.5)) {
                    break;
                }


                intakeState = true;


                if (verticalLiftValue > 3) {
                    verticalRight.setPower(-1);
                    verticalLeft.setPower(1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);

                    if (intakeInTransferPosition(wristServoController)) {
                        if (!magHorOn) { // 10 BEFORE CHANGE
                            horizontalDrive.setPower(-1);
                        } else {
                            horizontalDrive.setPower(0);


                            setPathState(106);
                            timeStamp = opmodeTimer.getElapsedTimeSeconds();

                            scoreSub = follower.pathBuilder().addPath(new BezierCurve(new Point(follower.getPose()), /* Control Point */ new Point(parkScoreControlPose), new Point(scorePose))).setTangentHeadingInterpolation().setReversed(true).build();

                            follower.followPath(scoreSub, true);
                        }
                    }
                }
                break;
            case 106:
                deposExtendo.setPosition(0.43);
                // transfer
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.1)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = true;
                    setPathState(107);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 107:
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.25)) { // 0.3 BEFORE changed //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(108);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 108:
                deposClawState = true;
                intakeClawState = false;
                intakeRotateState = false;

                deposExtendo.setPosition(0.41);


                /*
                scoreSub = new Path(new BezierCurve(new Point(follower.getPose()), new Point(parkScoreControlPose), new Point(scorePose)));
                scoreSub.setLinearHeadingInterpolation(parkPose.getHeading(), scorePose.getHeading());

                follower.followPath(scoreSub, true);
                 */


                setPathState(109);
                break;
            case 109:
                if (verticalLiftValue < 950) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                }

                deposWait = true;

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(110);

                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 110:
                deposWait = false;
                deposArmState = true;

                if (verticalLiftValue < 950) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                } else {
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                }

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = false;
                    setPathState(111);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 111:
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.15)) { // 0.3 before CHANGED
                    break;
                }
                deposClawState = true;
                numberScored++;
                timeStamp = opmodeTimer.getElapsedTimeSeconds();
                setPathState(112);
                break;
            case 112:
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
            if (runtime.seconds() > grabTimer + 0.7 && grabbing) {
                grabbing = false;
                intakeRotateOverride = false;
            }

            if (runtime.seconds() > grabTimer + 0.3 && grabbing && runtime.seconds() < grabTimer + 0.6) {
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

        if (deposClawState) { deposClaw.setPosition(0.8); } else { deposClaw.setPosition(0.3); }

        if (deposWait) {
            moveDeposTo("Wait", deposArm);
        } else if (deposArmState) {
            switch (scoreState) {
                case "Sample":
                    moveDeposTo("Depos", deposArm);
                    break;
                case "Specimen":
                    moveDeposTo("Specimen", deposArm);
                    break;
            }
        } else {
            if (scoreState.equals("Sample")) {
                moveDeposTo("Transfer", deposArm);
            } else {
                moveDeposTo("Depos Spec", deposArm);
            }
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
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

        // p 0.011 works, testing 0.013
        extendoController = new MotorPIDController(horizontalDrive, 0.011, 0, 0.000475, 0.1, (double) (700 / 180), 145.5, 4.941);
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
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();

        timeStamp = opmodeTimer.getElapsedTimeSeconds();

        // These lines for SUBMERSIBLE GRAB
        //setPathState(102);
        setPathState(0);
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
                    arm.setPosition(0.59);
                }
                break;
            case "Close": // Equal to transfer position
                arm.setPosition(0.9); // 0
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
                wrist.setPosition(0.25); // 0.15 earlier test
                break;
            case "Grab":
                wrist.setPosition(0.705);
                break;
        }
    }


    public void moveDeposTo(String state, Servo arm) {
        switch (state) {
            case "Transfer": // Equal to grab position
                arm.setPosition(0.235);
                break;
            case "Depos Spec":
                arm.setPosition(0.3);
                break;
            case "Depos": // Equal to transfer position
                arm.setPosition(0.765);
                break;
            case "Specimen":
                arm.setPosition(0.81);
                break;
            case "Wait":
                arm.setPosition(0.5);
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

