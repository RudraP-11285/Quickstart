package pedroPathing.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name="Encoder Test", group="TeleOp")
public class encoderTest extends LinearOpMode {
    private AnalogInput armEncoder1; // First wire
    private AnalogInput deposEncoder1; // First wire
    private AnalogInput wristEncoder1; // First wire
    private DigitalChannel limitSwitch;
    private Servo intakeArm =  null; // Servo that rotates the claw up down
    private Servo intakeWrist =  null; // Servo that rotates the claw up down
    private Servo intakeRotate =  null; // Servo that rotates the claw left right
    private Servo rotate =  null; // Servo that rotates the claw up down
    private Servo deposLeft =  null; // Servo that rotates the claw up down
    private Servo deposRight =  null; // Servo that rotates the claw up downprivate Servo
    private Servo claw =  null; // Servo that rotates the claw up down
    private DcMotor verticalDrive = null;
    private ColorSensor colorSensor = null;
    private Servo indicatorServo;
    private Limelight3A limelight = null;

    int state = 0;
    boolean stateDebounce = false;

    boolean calculateDebounce = false;

    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P3
        deposLeft = hardwareMap.get(Servo.class, "deposLeft"); // Exp. Hub P3
        deposRight = hardwareMap.get(Servo.class, "deposRight"); // Exp. Hub P3

        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist"); // Exp. Hub P3
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P2
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P1

        claw = hardwareMap.get(Servo.class, "intakeClaw"); // Exp. Hub P3
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        indicatorServo = hardwareMap.get(Servo.class, "indicator_servo");

        rotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P3
        armEncoder1 = hardwareMap.get(AnalogInput.class, "armEncoder1");
        deposEncoder1 = hardwareMap.get(AnalogInput.class, "depositEncoder1");
        wristEncoder1 = hardwareMap.get(AnalogInput.class, "wristEncoder1");
        verticalDrive = hardwareMap.get(DcMotor.class, "horizontalDrive");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimHorizontal1"); // 'magLimVert1' is the name in the config file
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");

        /*
        limelight.setPollRateHz(100);
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.reloadPipeline();
         */

        state = 0;
        stateDebounce = false;

        waitForStart();


        while (opModeIsActive()) {
            intakeArm.setPosition(0.7);
            intakeWrist.setPosition(1);
            intakeRotate.setPosition(0.72);

            float bluePercent = (float) colorSensor.blue() / (colorSensor.blue() + colorSensor.red() + colorSensor.green());
            float redPercent = (float) colorSensor.red() / (colorSensor.blue() + colorSensor.red() + colorSensor.green());
            float greenPercent = (float) colorSensor.green() / (colorSensor.blue() + colorSensor.red() + colorSensor.green());

            if (bluePercent > 0.40) {
                indicatorServo.setPosition(0.611);
            } else if (redPercent > 0.35 && greenPercent > 0.35) {
                indicatorServo.setPosition(0.388);
            } else if (redPercent > 0.40) {
                indicatorServo.setPosition(0.279);
            } else {
                indicatorServo.setPosition(1);
            }


            boolean isPressed = !limitSwitch.getState(); // Usually, "false" means pressed

            // Display the state on the telemetry
            telemetry.addData("Limit Switch Pressed", isPressed);
            telemetry.addData("Horizontal Drive Position: ", verticalDrive.getCurrentPosition());

            telemetry.addData("Intake Arm Encoder in Degrees:", (armEncoder1.getVoltage() / armEncoder1.getMaxVoltage()) * 360.0);
            telemetry.addData("Depos Arm Encoder in Degrees:", (deposEncoder1.getVoltage() / deposEncoder1.getMaxVoltage()) * 360.0);
            telemetry.addData("Wrist Encoder in Degrees:", (wristEncoder1.getVoltage() / wristEncoder1.getMaxVoltage()) * 360.0);

            telemetry.addData("Color Sensor Blue:", colorSensor.blue());
            telemetry.addData("Color Sensor Red:", colorSensor.red());
            telemetry.addData("Color Sensor Green:", colorSensor.green());

            telemetry.addData("Color Sensor Blue, %:", bluePercent);
            telemetry.addData("Color Sensor Red, %:", redPercent);
            telemetry.addData("Color Sensor Green, %:", greenPercent);

            telemetry.addData("Color Sensor:", colorSensor.argb());

            telemetry.addData("State:", state);

            telemetry.update();
        }
    }
}
