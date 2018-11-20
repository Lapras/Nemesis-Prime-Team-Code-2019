package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="roverRuckusTeleop", group="Linear Opmode")
public class teleOpFinal extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.subsystems.driveTrain dTrain = new org.firstinspires.ftc.teamcode.subsystems.driveTrain();
    public org.firstinspires.ftc.teamcode.subsystems.intake intake = new org.firstinspires.ftc.teamcode.subsystems.intake();
    public org.firstinspires.ftc.teamcode.subsystems.hardwareMap hwMap= new org.firstinspires.ftc.teamcode.subsystems.hardwareMap();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor intakeDrive;
    private Servo dropServo;
    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
        dropServo = hardwareMap.get(Servo.class, "drop_Servo");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        double intakePower = 0;
        double speedlimiter = 0;
        while (opModeIsActive()) {
            if (gamepad1.y) {
                speedlimiter = 1.0;
            } else {
                speedlimiter = 0.75;
            } if (gamepad2.a || gamepad2.b || gamepad1.left_trigger >= 0.2 || gamepad1.right_trigger >= 0.2) {
                if (gamepad2.a || gamepad1.right_trigger >= 0.2){
                    intakePower = 1.0;
                } else if (gamepad2.b || gamepad1.left_trigger >= 0.2) {
                    intakePower = -1.0;
                }
            } else {
                intakePower = 0.0;
            }
            if (gamepad1.y) {
                speedlimiter = 1.0;
            } else {
                speedlimiter = 0.5;
            } if (gamepad2.a || gamepad2.b || gamepad1.left_trigger >= 0.2 || gamepad1.right_trigger >= 0.2) {
                if (gamepad1.right_trigger >= 0.2){
                    intakePower = 1.0;
                } else if (gamepad1.left_trigger >= 0.2) {
                    intakePower = -1.0;
                }
            } else {
                intakePower = 0.0;
            }
            if (gamepad1.a) {
                dropServo.setPosition(1.0);
            } else if (gamepad1.b) {
                dropServo.setPosition(0.0);
            }
            dTrain.teleopInput(gamepad1.left_stick_y,-gamepad1.right_stick_x,speedlimiter, leftDrive, rightDrive);
            intake.teleopIntake(intakePower, intakeDrive);

        }
    }
}
