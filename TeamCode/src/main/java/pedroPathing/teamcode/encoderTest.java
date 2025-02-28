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
        verticalDrive = hardwareMap.get(DcMotor.class, "leftRear");
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
            verticalDrive.setPower(gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
