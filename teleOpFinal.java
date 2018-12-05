package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="roverRuckusTeleop", group="Linear Opmode")
public class teleOpFinal extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.subsystems.driveTrain dTrain = new org.firstinspires.ftc.teamcode.subsystems.driveTrain();
    public org.firstinspires.ftc.teamcode.subsystems.intake intake = new org.firstinspires.ftc.teamcode.subsystems.intake();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor elevatorDrive = null;
    static final double COUNTS_PER_MOTOR_REV_ELAVATOR = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ELAVATOR = 2.0;     // This is < 1.0 if geared UP
    static final double SPROCKET_TEETH = 20;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_ELAVATOR * DRIVE_GEAR_REDUCTION_ELAVATOR) /
            (SPROCKET_TEETH * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel digitalTouch = null;
    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        runtime.reset();
        double speedlimiter = 0;
        while (opModeIsActive()) {
            if (gamepad1.y) {
                speedlimiter = 1.0;
            } else {
                speedlimiter = 0.5;
            }
            boolean elevatormoveUP = gamepad2.a;
            boolean elevatormoveDOWN = gamepad2.b;
            boolean elevatorSTOP = gamepad2.x;
            if (elevatormoveUP) {
                elavatorMove(5, 4, 2);
            } else if (elevatormoveDOWN) {
                elavatorMove(5,-4,2);
            } else if (elevatorSTOP) {
                elevatorDrive.setPower(0);
            } else telemetry.addData("what did you do xD", null);
            teleopInput(gamepad1.left_stick_y,-gamepad1.right_stick_x,speedlimiter, leftDrive, rightDrive);

        }
    }
    public void teleopInput(double drive, double turn, double speedLimiter, DcMotor leftDrive, DcMotor rightDrive) {
        double leftPower = 0;
        double rightPower = 0;
        leftPower = Range.clip(drive + turn, -speedLimiter, speedLimiter);
        rightPower = Range.clip(drive - turn, -speedLimiter, speedLimiter);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    public void elavatorMove(double speed, double inches, double timeoutS) {
        int newTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = elevatorDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            elevatorDrive.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION

            elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            elevatorDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                if(digitalTouch.getState()){
                    elevatorDrive.setPower(0);
                }
            }
            // Stop all motion;
            elevatorDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
}
