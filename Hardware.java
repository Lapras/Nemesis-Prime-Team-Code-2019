package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
@Disabled
@Autonomous(name = "Hardware", group = "Concept")
public class Hardware  extends LinearOpMode {
        public void runOpMode() {
        }
        public void hardwareMap () {
            leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
            Markerservo = hardwareMap.get(Servo.class, "servo");
            intakeServo = hardwareMap.get (Servo.class, "intake");
            linearSlide = hardwareMap.get (DcMotor.class, "linear slider");
            //gyroParams
            BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
            parametersGyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parametersGyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parametersGyro.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parametersGyro.loggingEnabled = true;
            parametersGyro.loggingTag = "IMU";
            parametersGyro.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            //motor polarity
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
            linearSlide.setDirection(DcMotor.Direction.FORWARD);
            elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gyro.initialize(parametersGyro);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        }
    //Variables
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    BNO055IMU gyro;
    Servo markerServo;
    DcMotor elevatorDrive = null;
    Servo Markerservo = null;
    Servo intakeServo = null;
    DcMotor linearSlide = null;
    //gyroParams
}
