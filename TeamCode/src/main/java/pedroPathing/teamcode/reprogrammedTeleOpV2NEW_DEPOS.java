/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package pedroPathing.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Into the Deep - V2 TeleOp DEPOS v3", group="TeleOp")
public class reprogrammedTeleOpV2NEW_DEPOS extends LinearOpMode {

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

//    private Servo deposClaw =  null; // Edward
//    private Servo deposLeft =  null; // Stuart
//    private Servo deposRight =  null; // Felicia

    // NEW DEPOS ASSEMBLY SERVOS
    private Servo servoSlide =  null; // Edward
    private Servo inClaw =  null; // Stuart
    private Servo wristVert =  null; // Felicia

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

//    ContinuousServoController deposLeftController = null;
//    ContinuousServoController deposRightController = null;
//    ContinuousServoController wristServoController = null;
    //ContinuousServoController intakeArmServoController = null;

    // Sensors
    private DistanceSensor backDistance = null;
    private Limelight3A limelight;
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
    //endregion

    public enum ServoState {
        ON(0),
        OFF(1);
        private int stateIndex;
        private ServoState(int stateIndex) {
            this.stateIndex = stateIndex;
        }

        public int getStateIndex() {
            return stateIndex;
        }
    }

    public enum DeposState {
        START(new double[]{}),
        DEPOS1(new double[]{1.0,1.0,1.0,}), // Put servo values here
        DEPOS2(new double[]{0,0,0});  // Put other servo values here
        private double[] positions;
        private DeposState(double[] positions) {
            this.positions = positions;
        }

        public double[] getPositions() {
            return positions;
        }
    }
    NewArmControl.DeposState deposState = NewArmControl.DeposState.START;

    NewArmControl.ServoState vWristState = NewArmControl.ServoState.ON;
    NewArmControl.ServoState slideState = NewArmControl.ServoState.ON;
    NewArmControl.ServoState clawState = NewArmControl.ServoState.ON;
    //--------------------------------------------positions-----------------------------------------------
    private final double[] vWristPos = {0.0,1.0};
    private final double[] slidePos = {0.0,1.0};
    private final double[] clawPos = {0.0,1.0};

    boolean vWristDebounce = false;
    boolean slideDebounce = false;
    boolean newClawDebounce = false;
    boolean deposDebounce = false;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

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
//        deposClaw = hardwareMap.get(Servo.class, "deposClaw");
//        deposLeft = hardwareMap.get(Servo.class, "deposLeft");
//        deposRight = hardwareMap.get(Servo.class, "deposRight");

        servoSlide = hardwareMap.get(Servo.class,"deposExtendo");
        inClaw = hardwareMap.get(Servo.class,"deposClaw");
        wristVert = hardwareMap.get(Servo.class,"deposArm");

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.reloadPipeline();
        //endregion

        //region Set States
        String robotState = "Transfer";
        String scoreState = "Sample";
        //endregion

        // ########################################################################################
        // !!!!            IMPORTANT Drive Information. Test your motor directions.            !!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

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
        ContinuousServoController deposLeftController = new ContinuousServoController(dummy, depositEncoder1);
        ContinuousServoController deposRightController = new ContinuousServoController(dummy, depositEncoder1);
        ContinuousServoController wristServoController = new ContinuousServoController(dummy, wristEncoder1);
        //ContinuousServoController intakeArmServoController = new ContinuousServoController(intakeArm, armEncoder1);
        //endregion

        //region Wait for Start
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intakeArm.setPosition(0);

        waitForStart();
        runtime.reset();
        //endregion

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robotState = getRobotState(wristServoController, deposLeftController);

            //region Magnetic Limit Switches
            boolean magHorOn = !magLimHorizontal1.getState(); // Usually, "false" means pressed
            boolean magVertOn = !magLimVertical1.getState(); // Usually, "false" means pressed
            boolean limitSwitchNotTriggered = magLimVertical1.getState();
            //endregion

            //region Drivetrain Movement
            double lateralBoost = 0;
            if (gamepad1.dpad_left) {
                lateralBoost = -0.4;
            } else if (gamepad1.dpad_right) {
                lateralBoost = 0.4;
            }
            double axialBoost = 0;
            if (gamepad1.dpad_down) {
                axialBoost = -0.25;
            } else if (gamepad1.dpad_up) {
                axialBoost = 0.25;
            }

            double speedMultiplier = 0.67289;
            double max;

            if (gamepad1.left_bumper) {
                speedMultiplier = 0.25;
            } else if (gamepad1.right_bumper) {
                speedMultiplier = 1;
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = axialBoost - gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x + lateralBoost;
            double yaw     =  gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double upDrivePower    = 0;
            double outDrivePower   = 0;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            //endregion

            //region Return to Transfer
            // Bring everything back to the transfer position
            if (gamepad2.b) {
                intakeState = true;
                intakeRotateState = false;
                intakeClawState = true;
                deposArmState = false;


                if ((horizontalDrive.getCurrentPosition() > 350 && verticalRight.getCurrentPosition() > 25) || (!magHorOn && verticalRight.getCurrentPosition() < 25))  {
                    outDrivePower = -1;
                } else {
                    outDrivePower = 0;
                }

                if (verticalRight.getCurrentPosition() > 5) {
                    upDrivePower = -1;
                    //verticalLeft.setPower(1);
                    //verticalRight.setPower(-1);
                } else {
                    upDrivePower = 0;
                    //verticalLeft.setPower(0);
                    //verticalRight.setPower(0);
                }

                if (verticalRight.getCurrentPosition() < 200) {
                    deposClawState = false;
                    intakeWaitToReturn = false;
                } else {
                    intakeWaitToReturn = true;
                }
            }
            //endregion

            //region Intake Controls (Arm + Wrist)
            // NOTE: All code below controls the intake
            if (intakeState) {
                moveWristTo("Close", intakeWrist);
                if (Math.abs(wristServoController.getCurrentPositionInDegrees() - 50) <= 10) {
                    if (intakeWaitToReturn) {
                        //intakeClawState = true;
                        moveArmTo("Wait", intakeArm);
                    } else {
                        moveArmTo("Close", intakeArm);
                    }
                }
            } else {
                deposArmState = true;
                if (!grabbing) {
                    moveWristTo("Open", intakeWrist);
                } else {
                    moveWristTo("Grab", intakeWrist);
                }

                if ((Math.abs(wristServoController.getCurrentPositionInDegrees() - 9.16) <= 9.16) || (Math.abs(wristServoController.getCurrentPositionInDegrees() - 100) <= 10)) {
                    if (!grabbing) {
                        //intakeClawState = false;
                        moveArmTo("Open", intakeArm);
                    } else {
                        moveArmTo("Grab", intakeArm);
                    }
                }
            }
            if (gamepad2.a && (!intakeDebounce)) {
                intakeDebounce = true;
                intakeState = !intakeState;
            }
            if (!gamepad2.a && intakeDebounce) {
                intakeDebounce = false;
            }
            //endregion

            //region Intake Rotate Controls
            // NOTE: All code below controls the intake rotate
            // intakeRotateState = true (Not Transfer Position) or false (Transfer Position)
            if (intakeRotateState) { intakeRotate.setPosition(0); } else { intakeRotate.setPosition(1); }
            // Intake Rotate controls under different circumstances
            switch (robotState) {
                case "Transfer Ready":
                    // When transferring, lock claw rotation in transfer position
                    intakeRotateState = false;
                    break;
                case "Transfer Complete":
                    // When transfer is done, keep it locked to avoid interference
                    intakeRotateState = false;
                    break;
                default:
                    if (gamepad2.right_bumper && (!intakeRotateDebounce) && !gamepad2.start) {
                        intakeRotateDebounce = true;
                        // In all other cases, allow free control
                        intakeRotateState = !intakeRotateState;
                    }
                    break;
            }
            // Reset debounce once key up
            if (!gamepad2.right_bumper && intakeRotateDebounce) {
                intakeRotateDebounce = false;
            }
            //endregion

            //region Intake Claw Controls
            // NOTE: All code below controls the intake claw
            LLResult cameraResult = limelight.getLatestResult();
            double[] pythonOutputs = cameraResult.getPythonOutput();

            // intakeClawState = true (Closed) or false (Open)
            if (intakeClawState) { intakeClaw.setPosition(0); } else { intakeClaw.setPosition(1); }
            // Claw controls under different circumstances
            switch (robotState) {
                case "Grab":
                    if (gamepad2.x && (!intakeClawDebounce)) {
                        intakeClawDebounce = true;
                        if (intakeClawState) {
                            intakeClawState = false;
                        } else {
                            grabTimer = runtime.seconds();
                            //intakeClawState = true;
                            grabbing = true;
                        }
                        // If "x" pressed while grabbing, jab down and grab. Otherwise allow open and close
                    } else if (autoIntakeMode && (pythonOutputs[0] > 0.5) && (Math.abs(pythonOutputs[1]) < 120 && Math.abs(pythonOutputs[2]) < 60) && !intakeClawState && !grabbing && (!intakeClawDebounce)) {
                        grabTimer = runtime.seconds();
                        //intakeClawState = true;
                        grabbing = true;


                        if (pythonOutputs[4] >0.5) {
                            intakeRotateState = true;
                        } else {
                            intakeRotateState = false;
                        }

                    } else if (autoIntakeMode && (pythonOutputs[5] > 0.5) && (Math.abs(pythonOutputs[6]) < 120 && Math.abs(pythonOutputs[7]) < 60) && !intakeClawState && !grabbing && (!intakeClawDebounce)) {
                        grabTimer = runtime.seconds();
                        //intakeClawState = true;
                        grabbing = true;


                        if (pythonOutputs[9] > 0.5) {
                            intakeRotateState = true;
                        } else {
                            intakeRotateState = false;
                        }

                    }
                    break;
                case "Transfer Ready":
                    // Do not allow opening. Keep it closed.
                    intakeClawState = true;
                    break;
                default:
                    if (gamepad2.x && (!intakeClawDebounce)) {
                        intakeClawDebounce = true;
                        // In all other cases, allow free control
                        intakeClawState = !intakeClawState;
                    }
                    break;
            }
            // Reset debounce once key up
            if (!gamepad2.x && intakeClawDebounce) {
                intakeClawDebounce = false;
            }
            if (autoIntakeMode) {
                if (runtime.seconds() > grabTimer + 0.4 && grabbing) {
                    grabbing = false;
                }
            } else {
                if (runtime.seconds() > grabTimer + 0.3 && grabbing) {
                    grabbing = false;
                }
            }
            if (runtime.seconds() > grabTimer + 0.01 && grabbing && runtime.seconds() < grabTimer + 0.6) {
                intakeClawState = true;
            }
            //endregion

            //region Horizontal Drive Controls
            // LOCK THE OUT DRIVE!
            if (gamepad1.x && (!horizontalDriveLockDebounce)) {
                horizontalDriveLockDebounce = true;
                if (horizontalDriveLockState) {
                    // Lock The Out Drive
                    horizontalDriveLockState = false;
                    /*
                    outDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outDrive.setTargetPosition(outDrive.getCurrentPosition());
                     */
                } else {
                    // Unlock the Out Drive
                    horizontalDriveLockState = true;
                    horizontalDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // (un)Rotate the servo
                }
            }
            if (!gamepad1.x && horizontalDriveLockDebounce) {
                horizontalDriveLockDebounce = false;
            }


            // Horizontal "Lift" Motor Controls
            if (!gamepad2.b) {
                if (gamepad2.dpad_right) {
                    outDrivePower = 1;
                } else if (gamepad2.dpad_left && !magHorOn) {
                    outDrivePower = -1;
                } else {
                    outDrivePower = 0;
                }
            }

            /*
            if ((magHorOn) && (!gamepad2.dpad_right) && (!horizontalDriveLockState)) {
                horizontalDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalDrive.setTargetPosition(0);
                outDrivePower = 0.2;
            } else if (horizontalDriveLockState) {
                horizontalDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalDrive.setTargetPosition(0);
                outDrivePower = 0.2;
            } else {
                horizontalDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            */



            // Send calculated power to wheels
            if (!magHorOn) {
                outDrivePower += (gamepad2.right_trigger - gamepad2.left_trigger);
            } else {
                outDrivePower += (gamepad2.right_trigger);
            }
            //endregion

            //region Depos Claw Controls
//            if (deposClawState) { deposClaw.setPosition(0.8); } else { deposClaw.setPosition(0.3); }
//            switch (robotState) {
//                case "Depos":
//                    if (deposArmDown(deposLeftController)) {
//                        deposClawState = true;
//                    } else {
//                        if (gamepad2.y && (!deposClawDebounce)) {
//                            deposClawDebounce = true;
//                            deposClawState = !deposClawState;
//                        }
//                    }
//                    break;
//                case "Stop Giving Me Errors":
//                    break;
//                default:
//                    if (gamepad2.y && (!deposClawDebounce)) {
//                        deposClawDebounce = true;
//                        deposClawState = !deposClawState;
//                    }
//                    break;
//            }
//            if (!gamepad2.y && deposClawDebounce) {
//                deposClawDebounce = false;
//            }
            //endregion

            //region Depos Arm Controls
//            if (gamepad2.left_bumper && (!deposArmDebounce)) {
//                deposArmDebounce = true;
//                deposArmState = !deposArmState;
//            }
//            if (!(gamepad2.left_bumper || gamepad1.y || gamepad2.left_stick_button) && deposArmDebounce) {
//                deposArmDebounce = false;
//            }
//            if (deposArmState) {
//                switch (scoreState) {
//                    case "Sample":
//                        moveDeposTo("Depos", deposLeft, deposRight);
//                        break;
//                    case "Specimen":
//                        moveDeposTo("Specimen", deposLeft, deposRight);
//                        break;
//                }
//            } else {
//                moveDeposTo("Transfer", deposLeft, deposRight);
//            }

            boolean[] gamepadBools = {gamepad2.left_bumper,gamepad2.right_bumper,gamepad2.dpad_up};
            NewArmControl.ServoState[] servoStates = {slideState,vWristState,clawState};
            boolean[] servoDebounces = {slideDebounce,vWristDebounce,newClawDebounce};


            Servo[] intakeServos = {servoSlide,wristVert,inClaw};

            for (int i=0;i<3;i++) {
                if (!gamepadBools[i]) {
                    servoDebounces[i] = false;
                } else if (gamepadBools[i] && !servoDebounces[i] && gamepad2.start) {
                    switch (servoStates[i]) {
                        case ON:
                            servoStates[i] = NewArmControl.ServoState.OFF;
                            break;
                        case OFF:
                            servoStates[i] = NewArmControl.ServoState.ON;
                            break;
                    }
                    intakeServos[i].setPosition(slidePos[servoStates[i].getStateIndex()]);
                    servoDebounces[i] = true;
                }
            }

            if (!gamepad2.y && deposDebounce) {
                deposDebounce = false;
            } else if (gamepad2.y && !deposDebounce) {
                switch (deposState) {
                    case DEPOS1:
                        deposState = NewArmControl.DeposState.DEPOS2;
                        break;
                    case DEPOS2:
                        deposState = NewArmControl.DeposState.DEPOS1;
                        break;
                }
                for (int i = 0;i<3;i++) {
                    intakeServos[i].setPosition(deposState.getPositions()[i]);
                }
                deposDebounce = true;
            }

            if ((gamepad1.y || gamepad2.left_stick_button) && (!deposArmDebounce)) {
                deposArmDebounce = true;
                switch (scoreState) {
                    case "Sample":
                        scoreState = "Specimen";
                        break;
                    case "Specimen":
                        scoreState = "Sample";
                        break;
                }
            }
            //endregion

            //region Vertical Lift Controls
            // Vertical Lift Motor Controls
            if (gamepad2.dpad_up && !gamepad2.start) {
                upDrivePower = 1;
            } else if (gamepad2.dpad_down) {
                upDrivePower = -1;
            } else if (!gamepad2.b) {
                upDrivePower = 0;
            }


            if (magVertOn && (upDrivePower < 0)) { // Negate downward movement if limit is active
                upDrivePower = 0;
            } else if ((verticalRight.getCurrentPosition() > 3050) && (upDrivePower > 0)) { // Negate upward movement if too high
                upDrivePower = 0;
            }
            //endregion

            if (gamepad1.b && (!autoIntakeDebounce)) {
                autoIntakeDebounce = true;
                autoIntakeMode = !autoIntakeMode;
            }
            if (!gamepad1.b && autoIntakeDebounce) {
                autoIntakeDebounce = false;
            }

            // Everything above are controls
            // Everything below runs motors and telemetry

            //region Run Motors
            leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
            rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
            leftBackDrive.setPower(leftBackPower * speedMultiplier);
            rightBackDrive.setPower(rightBackPower * speedMultiplier);
            //if (!gamepad2.b) {
            verticalRight.setPower(upDrivePower);
            verticalLeft.setPower(-upDrivePower);
            horizontalDrive.setPower(outDrivePower);
            //}
            //endregion

            //region Telemetry
            int upDrivePos1 = verticalRight.getCurrentPosition();
            int upDrivePos2 = verticalLeft.getCurrentPosition();
            LLResult result = limelight.getLatestResult();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift Encoder Values: ", upDrivePos1 + ", " + upDrivePos2);

            telemetry.addData("Robot State", robotState);
            telemetry.addData("Claw State", intakeClawState);
            telemetry.addData("Intake State", intakeState);


            //telemetry.addData(" ", " ");
            //telemetry.addData("Robot State: ", robotState);
            //telemetry.addData("Intake Rotate State: ", intakeRotateState);
            //telemetry.addData("Intake Claw State: ", intakeClawState);
            //telemetry.addData("Depos Arm State: ", deposArmState);
            //telemetry.addData("Depos Wait: ", intakeWaitToReturn);
            //telemetry.addData("Horizontal Drive Position: ", horizontalDrive.getCurrentPosition());


            //telemetry.addData(" ", " ");
            telemetry.addData("Wrist Servo Encoder: ", (wristServoController.getCurrentPositionInDegrees()));
            telemetry.addData("Arm Servo: ", (intakeArm.getPosition()));
            //telemetry.addData("Depos Servo Encoder: ", (deposLeftController.getCurrentPositionInDegrees()));


            telemetry.addData("Connected?", limelight.isConnected());
            telemetry.addData("Running?", limelight.isRunning());

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("Blue Vertical",pythonOutputs[4]);
            telemetry.addData("Yellow Vertical", pythonOutputs[9]);


            if (result != null) {
                telemetry.addData("PythonOutput", Arrays.toString(result.getPythonOutput()));
            } else {
                telemetry.addData("PythonOutput", "nothing.");
            }

            telemetry.update();
            //endregion
        }
        limelight.stop();
    }


    //region Servo Helper Functions
    // Move intake wrist to "Open" or "Close"
    public void moveArmTo(String state, Servo arm) {
        switch (state) {
            case "Open": // Equal to grab position
                if (autoIntakeMode) {
                    arm.setPosition(0.4);
                } else {
                    arm.setPosition(0.55);
                }
                break;
            case "Close": // Equal to transfer position
                arm.setPosition(0);
                break;
            case "Grab": // Equal to grab position
                arm.setPosition(0.675);
                break;
            case "Wait": // Wait to return position
                arm.setPosition(0.4);
                break;
        }
    }

    // Move intake wrist to "Open" or "Close"
    public void moveWristTo(String state, Servo wrist) {
        switch (state) {
            case "Open": // Equal to grab position
                if (autoIntakeMode) {
                    wrist.setPosition(0);
                } else {
                    wrist.setPosition(0.25);
                }
                break;
            case "Close": // Equal to transfer position
                wrist.setPosition(1);
                break;
            case "Grab":
                wrist.setPosition(0.275);
                break;
        }
    }


    public void moveDeposTo(String state, Servo left, Servo right) {
        switch (state) {
            case "Transfer": // Equal to grab position
                left.setPosition(1);
                right.setPosition(0);
                break;
            case "Depos": // Equal to transfer position
                left.setPosition(0.56);
                right.setPosition(0.44);
                break;
            case "Specimen":
                left.setPosition(0.3);
                right.setPosition(0.7);
                break;
        }
    }
    //endregion

    //region State Finding Functions
    // Check if horizontal slides are all the way in
    public Boolean extendoClosed() { return (horizontalDrive.getCurrentPosition() < 25); }
    // Check if lift is all the way down
    public Boolean liftDown() { return (verticalRight.getCurrentPosition() < 25); }
    // Check if wrist and arm are back and claw is rotated in transfer position
    public Boolean intakeInTransferPosition(ContinuousServoController controllerWrist) { return (Math.abs(controllerWrist.getCurrentPositionInDegrees() - 60.5) < 5); }
    // Check if the depos arm is down
    public Boolean deposArmDown(ContinuousServoController controllerDepos) { return (Math.abs(controllerDepos.getCurrentPositionInDegrees() - 14) < 3); }
    // Check if the depos claw is closed
    public Boolean deposClawClosed() { return deposClawState; }

    // Get the state of the robot based on other values; can be overridden by certain controls
    public String getRobotState(ContinuousServoController controllerWrist, ContinuousServoController controllerDepos) {
        if (extendoClosed() && liftDown() && deposArmDown(controllerDepos) && intakeInTransferPosition(controllerWrist) && !deposClawClosed()) {
            // If everything retracted and depos claw open, basically starting position
            return "Transfer Ready";
        } else if (extendoClosed() && liftDown() && deposArmDown(controllerDepos) && intakeInTransferPosition(controllerWrist) && deposClawClosed()) {
            // If everything retracted and depos claw closed, basically starting position
            return "Transfer Complete";
        } else if (!extendoClosed() || !intakeInTransferPosition(controllerWrist)) {
            // If extendo not in and arm/wrist not retracted
            return "Grab";
        } else if (!liftDown() || !deposArmDown(controllerDepos)) {
            // If lift not down
            return "Depos";
        }
        return "Unknown";
    }
    //endregion
}
