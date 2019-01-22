package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Variables;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.List;
@Disabled
@Autonomous(name = "Methods", group = "Concept")
public class Methods  extends LinearOpMode {
    public Variables Variables = new Variables();
    public Hardware Hardware = new Hardware();
    public void runOpMode() {
        Hardware.hardwareMap();
    }
        void initVuforia () {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = Variables.VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            Variables.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }

        void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            Variables.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, Variables.vuforia);
            Variables.tfod.loadModelFromAsset(Variables.TFOD_MODEL_ASSET, Variables.LABEL_GOLD_MINERAL, Variables.LABEL_SILVER_MINERAL);
        }

        public void gyroTurn ( double speed, double angle){
            double angleTarget;
            angleTarget = getHeading() - angle;
            while (opModeIsActive() && !onHeading(speed, angleTarget, Variables.P_TURN_COEFF)) {
                telemetry.update();
            }
        }

        public void gyroTurnTo ( double speed, double angle){
            while (opModeIsActive() && !onHeading(speed, angle, Variables.P_TURN_COEFF)) {
                telemetry.update();
            }
        }

        public void gyroHold ( double speed, double angle, double holdTime){
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {
                onHeading(speed, angle, Variables.P_TURN_COEFF);
                telemetry.update();
            }
            Hardware.leftDrive.setPower(0);
            Hardware.rightDrive.setPower(0);
        }

        boolean onHeading ( double speed, double angle, double PCoeff){
            double error;
            double steer;
            boolean onTarget = false;
            double leftSpeed;
            double rightSpeed;
            error = getError(angle);
            if (Math.abs(error) <= Variables.HEADING_THRESHOLD) {
                steer = 0.0;
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            } else {
                steer = getSteer(error, PCoeff);
                double signMultiplier = 0;
                if (steer > 0) {
                    signMultiplier = 1;
                } else if (steer < 0) {
                    signMultiplier = -1;
                }
                rightSpeed = signMultiplier * Range.clip(Math.abs(speed * steer), 0.2, 1);
                leftSpeed = -rightSpeed;
            }
            Hardware.leftDrive.setPower(leftSpeed);
            Hardware.rightDrive.setPower(rightSpeed);
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            telemetry.addData("heading", "%5.1f,", getHeading());
            telemetry.update();
            return onTarget;
        }

        public double getError ( double targetAngle){
            double robotError;
            robotError = targetAngle - getHeading();
            return robotError;
        }

        public double getSteer ( double error, double PCoeff){
            return Range.clip(error * PCoeff, -1, 1);
        }

        public double getHeading () {
            Orientation angles = Hardware.gyro.getAngularOrientation();
            return angles.firstAngle + 180;
        }

        public void encoderDrive ( double speed, double leftMM, double rightMM, double timeoutS){
            int newLeftTarget;
            int newRightTarget;
            if (opModeIsActive()) {
                newLeftTarget = Hardware.leftDrive.getCurrentPosition() - (int) (leftMM * Variables.COUNTS_PER_MM);
                newRightTarget = Hardware.rightDrive.getCurrentPosition() - (int) (rightMM * Variables.COUNTS_PER_MM);
                Hardware.leftDrive.setTargetPosition(newLeftTarget);
                Hardware.rightDrive.setTargetPosition(newRightTarget);
                Hardware.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Hardware. rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Variables.runtime.reset();
                Hardware.leftDrive.setPower(Math.abs(speed));
                Hardware. rightDrive.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (Variables.runtime.seconds() < timeoutS) &&
                        (Hardware.leftDrive.isBusy() && Hardware.rightDrive.isBusy())) {
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            Hardware.leftDrive.getCurrentPosition(),
                            Hardware.rightDrive.getCurrentPosition());
                    telemetry.update();
                }
                Hardware.leftDrive.setPower(0);
                Hardware.rightDrive.setPower(0);
                Hardware.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Hardware.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (opModeIsActive()) {
            newTarget = Hardware.elevatorDrive.getCurrentPosition() + (int) (inches * Variables.COUNTS_PER_INCH_ELEVATOR);
            Hardware.elevatorDrive.setTargetPosition(newTarget);
            Hardware.elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Hardware.elevatorDrive.setPower(Math.abs(1));
            Variables.runtime.reset();
            while (opModeIsActive() &&
                    (Variables.runtime.seconds() < timeoutS) &&
                    (Hardware.elevatorDrive.isBusy())) {
                telemetry.addData("Path1", "Running to %7d,", newTarget);
                telemetry.update();
            }

            Hardware.leftDrive.setPower(0);
            Hardware. rightDrive.setPower(0);
            Hardware. elevatorDrive.setPower(0);
            Hardware.  elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void elevatorDrive(double speed, double elevatorDis, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = Hardware.elevatorDrive.getCurrentPosition() + (int) (elevatorDis * Variables.COUNTS_PER_INCH_ELEVATOR);
            Hardware.elevatorDrive.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            Hardware.elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            Variables.runtime.reset();
            Hardware.elevatorDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (Variables.runtime.seconds() < timeoutS) &&
                    (Hardware.elevatorDrive.isBusy())) {
                //if (digitalTouch.getState()) {
                //  elevatorDrive.setPower(0);
                //}
            }
            // Stop all motion;
            Hardware.elevatorDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            Hardware.elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
