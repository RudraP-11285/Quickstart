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

import pedroPathing.teamcode.ContinuousServoController;

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

@TeleOp(name="NEW ARM CONTROL for V:3", group="TeleOp")
public class NewArmControl extends LinearOpMode {

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

    ContinuousServoController deposLeftController = null;
    ContinuousServoController deposRightController = null;
    ContinuousServoController wristServoController = null;
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
        DEPOS2(new double[]{1.0,1.0,1.0});  // Put other servo values here
        private double[] positions;
        private DeposState(double[] positions) {
            this.positions = positions;
        }

        public double[] getPositions() {
            return positions;
        }
    }
    DeposState deposState = DeposState.START;

    ServoState vWristState = ServoState.ON;
    ServoState slideState = ServoState.ON;
    ServoState clawState = ServoState.ON;

    private final double[] vWristPos = {0.78,0.725};
    private final double[] slidePos = {0.36,0.585};
    private final double[] clawPos = {0.71,0.45};

    boolean vWristDebounce = false;
    boolean slideDebounce = false;
    boolean newClawDebounce = false;
    boolean transferDebouce = false;
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
//-----------------------------------New Servos------------------------------------------
        servoSlide = hardwareMap.get(Servo.class,"deposExtendo");
        inClaw = hardwareMap.get(Servo.class,"deposClaw");
        wristVert = hardwareMap.get(Servo.class,"deposArm");
//---------------------------------------------------------------------------------------

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

            int[] currentIndices = {0,0,0};
            boolean[] gamepadBools = {gamepad2.left_bumper,gamepad2.right_bumper,gamepad2.a};
            ServoState[] servoStates = {slideState,vWristState,clawState};
            boolean[] servoDebounces = {slideDebounce,vWristDebounce,newClawDebounce};
            double[][] servoPositions = {slidePos,vWristPos,clawPos};

            Servo[] intakeServos = {servoSlide,wristVert,inClaw};

            for (int i=0;i<3;i++) {
                if (!gamepadBools[i]) {
                    servoDebounces[i] = false;
                } else if (gamepad2.left_bumper && !servoDebounces[i]) {
                    switch (servoStates[i]) {
                        case ON:
                            servoStates[i] = ServoState.OFF;
                            break;
                        case OFF:
                            servoStates[i] = ServoState.ON;
                            break;
                    }
                    intakeServos[i].setPosition(servoPositions[i][servoStates[i].getStateIndex()]);
                    servoDebounces[i] = true;
                }
            }

//--------------------------------------------HERE-----------------------------------------------------
            if (robotState == "Depos") {

            }
            if (!gamepad2.y && transferDebouce) {
                transferDebouce = false;
            } else if (gamepad2.y && !transferDebouce) {
                switch (deposState) {
                    case DEPOS1:
                        deposState = DeposState.DEPOS2;
                        break;
                    case DEPOS2:
                        deposState = DeposState.DEPOS1;
                        break;
                }
                for (int i = 0;i<3;i++) {
                    intakeServos[i].setPosition(deposState.getPositions()[i]);
                }
                //Maybe put this outside the conditional
                transferDebouce = true;
            }

//-----------------------------------------------------------------------------------------------------
//            int currentIndex = 0;
//            if (!(gamepad2.left_bumper && gamepad2.right_bumper)) {
//                slideDebounce = false;
//            } else if (gamepad2.left_bumper && !slideDebounce) {
//                switch (slideState) {
//                    case ON:
//                        currentIndex = 0;
//                        slideState = ServoStates.OFF;
//                        break;
//                    case OFF:
//                        currentIndex = 1;
//                        slideState = ServoStates.ON;
//                        break;
//                }
//                servoSlide.setPosition(slidePos[currentIndex]);
//                slideDebounce = true;
//                }
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

    public DeposState currentDeposState(Servo[] servos) {
        DeposState[] deposStates = new DeposState[]{DeposState.DEPOS1,DeposState.DEPOS2};
        int count = 0;
        for (int i = 0;i<2;i++) {
            for (int j = 0; j < 3; j++) {
                if (Math.abs(servos[j].getPosition() - deposStates[i].getPositions()[j]) < 0.01) {
                    count++;
                }
            }
            if (count == 3) {
                return deposStates[i];
            }
        }
        return DeposState.START;
    }

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
