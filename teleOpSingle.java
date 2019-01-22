package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Methods;

import java.lang.reflect.Method;

@TeleOp(name="roverRuckusTeleopSingle", group="Linear Opmode")
public class teleOpSingle extends LinearOpMode {
    public Variables Variables = new Variables();
    public Methods Methods = new Methods();
    public Hardware Hardware = new Hardware();
    @Override
    public void runOpMode() {
        Hardware.hardwareMap();
        waitForStart();
        Variables.runtime.reset();
        double speedlimiter = 0;
        while (opModeIsActive()) {
            if (gamepad1.y) {
                speedlimiter = 1.0;
            } else {
                speedlimiter = 0.5;
            }
            if (gamepad1.a) {
                Hardware.elevatorDrive.setPower(1);
            } else if (gamepad1.b) {
                Hardware.elevatorDrive.setPower(-1);
            } else {
                Hardware.elevatorDrive.setPower(0);
            }
            if (gamepad1.right_trigger >= 0.2) {
                Hardware.linearSlide.setPower(1);
            } else if (gamepad1.left_trigger >= 0.2) {
                Hardware.linearSlide.setPower(-1);
            } else {
                Hardware.linearSlide.setPower(0);
            } if (gamepad1.x) {
                Variables.servoPos = !Variables.servoPos;
            } if (Variables.servoPos) {
                Hardware.intakeServo.setPosition(1);
            } else {
                Hardware.intakeServo.setPosition(0);
            }
            Methods.teleopInput(gamepad1.left_stick_y,gamepad1.right_stick_x,speedlimiter, Hardware.leftDrive, Hardware.rightDrive);
            telemetry.addData("elevatorPosition %7d", Hardware.elevatorDrive.getCurrentPosition());
            telemetry.addData("servoPosition %7d", Hardware.intakeServo.getPosition());
            telemetry.addData ("leftPower", Hardware.leftDrive.getPower());
            telemetry.addData ("rightPower", Hardware.rightDrive.getPower());
            telemetry.update();
            telemetry.update();
        }
    }
}
