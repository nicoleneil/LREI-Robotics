package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Auto LBSample + ObsPark", group="Robot")

public class AutoLBSample_and_Park extends LinearOpMode {

    // Declare OpMode members
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null; 
    private CRServo intake = null; 
    private Servo   wrist = null; 
    
    // Declare variables for essential hardware positions
    final double WRIST_FOLDED_IN            = 0.4444;
    final double WRIST_FOLDED_OUT           = 0.7888;
    final double INTAKE_COLLECT             = -1.0;
    final double INTAKE_OFF                 = 0.0;
    final double INTAKE_DEPOSIT             = 0.5;
    final double ARM_TICKS_PER_DEGREE       = 19.7924893140647; //exact fraction is (194481/9826)
    final double ARM_COLLAPSED_INTO_ROBOT   = 0;
    final double ARM_SCORE_LOW_SAMPLE       = 140 * ARM_TICKS_PER_DEGREE;
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor = FUDGE_FACTOR;

    static final double     FORWARD_SPEED = 0.4;
    static final double     TURN_SPEED    = 0.5;
    static final double     REVERSE_SPEED = -0.4;
    static final double     BRAKE = 0.0;

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set arm, wrist, and intake position to neutral for start of match
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
        wrist.setPosition(WRIST_FOLDED_IN);
        intake.setPower(INTAKE_OFF);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset Runtime - Used for time-based calculations in control 
        runtime.reset();

        /* Step 0: Position Bot Facing Net Zone 
        /*         Align bot's back wheels at edge of 4th/3rd Arena Square 
        *          Secure sample in intake */
        
        // Step 1: Drive right for .125 seconds to clear Arena Wall
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(REVERSE_SPEED);
        rightFrontDrive.setPower(REVERSE_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < 0.125)) {
            telemetry.addData("Right Drive", "Clear Arena Wall", runtime.seconds());
            telemetry.update();
        }
        
        // Step 2: Drive forward for 1.5 second to reach Net Zone 
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 0.125) && (runtime.seconds() < 1.625)) {
            telemetry.addData("Forward Path to Net Zone", "Prep for Low Basket Score", runtime.seconds());
            telemetry.update();
        }
        
        // Step 3: Drive left for .125 seconds to align arm with Low Basket
        leftFrontDrive.setPower(REVERSE_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(REVERSE_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 1.625) && (runtime.seconds() < 1.75)) {
            telemetry.addData("Left Drive", "Align Arm with Low Basket", runtime.seconds());
            telemetry.update();
        }

        // Step 4: Stop driving + Adjust arm position and fold wrist out (1.75 sec)
        leftFrontDrive.setPower(BRAKE);
        leftBackDrive.setPower(BRAKE);
        rightFrontDrive.setPower(BRAKE);
        rightBackDrive.setPower(BRAKE);
        
        armPosition = ARM_SCORE_LOW_SAMPLE;
        armMotor.setTargetPosition ((int) (armPosition + armPositionFudgeFactor));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(WRIST_FOLDED_OUT);

        while (opModeIsActive() && (runtime.seconds() > 1.75) && (runtime.seconds() < 3.5)) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Target Position", armMotor.getTargetPosition());
            telemetry.addData("Arm & Wrist Ready", "Prep for Low Basket Score", runtime.seconds());
            telemetry.update();
        }
        
        // Step 5: Deposit sample in Low Basket (1 sec)
        intake.setPower(INTAKE_DEPOSIT);

        while (opModeIsActive() && (runtime.seconds() > 3.5) && (runtime.seconds() < 4.5)) {
            telemetry.addData("Low Basket Sample Score", runtime.seconds());
            telemetry.update();
        }
        
        // Step 6: Revert arm, wrist, and intake to starting mode (1.75)
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
        wrist.setPosition(WRIST_FOLDED_IN);
        intake.setPower(INTAKE_OFF);
        
        while (opModeIsActive() && (runtime.seconds() > 4.5) && (runtime.seconds() < 6.25)) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Target Position", armMotor.getTargetPosition());
            telemetry.addData("Low Basket Sample Score", runtime.seconds());
            telemetry.update();
        }
        
        // Step 7: Drive right for 1.5 seconds to clear Alliance Bot (& Submersible)
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(REVERSE_SPEED);
        rightFrontDrive.setPower(REVERSE_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 6.25) && (runtime.seconds() < 7.75)) {
            telemetry.addData("Right Drive", "Clear Alliance Bot", runtime.seconds());
            telemetry.update();
        }

        // Step 8: Drive backwards for 5 seconds to reach Arena Back Wall
        leftFrontDrive.setPower(REVERSE_SPEED);
        leftBackDrive.setPower(REVERSE_SPEED);
        rightFrontDrive.setPower(REVERSE_SPEED);
        rightBackDrive.setPower(REVERSE_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 7.75) && (runtime.seconds() < 12.75)) {
            telemetry.addData("Backward Drive", "Reach Arena Back Wall", runtime.seconds());
            telemetry.update();
        }
 
        // Step 9: Drive left for 1.75 seconds to Park in Observation Zone
        leftFrontDrive.setPower(REVERSE_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(REVERSE_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 12.75) && (runtime.seconds() < 14.5)) {
            telemetry.addData("Left Drive", "Observation Zone Park", runtime.seconds());
            telemetry.update();
        }
        
        // Step 10: Stop Robot
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
