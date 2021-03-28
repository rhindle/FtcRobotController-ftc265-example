package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "LK_T265_Test", group = "")
//@Disabled
public class LKCamera extends LinearOpMode {

// New Camera Stuff
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //LK add from instructions
    //Transform2d cameraToRobot = new Transform2d();
    Transform2d cameraToRobot = new Transform2d(new Translation2d(-7 * 0.0254, 0.0 * 0.254), new Rotation2d());

    double encoderMeasurementCovariance = 0.1; //0.8;
    Pose2d startingPose = new Pose2d(-63 * 0.0254, 57 * 0.0254, new Rotation2d());
// End New Camera Stuff

    private BNO055IMU imu;
    private DcMotor motor0;
    private DcMotor motor2;
    private DcMotor motor1;
    private DcMotor motor3;

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    private double globalHeading;
    private double storedHeading = 0;
    private double deltaHeading = 0;
    private int toggleRotate;

    private final double maxSpeed = 1;//0.2;

    /**
     * Describe this function...
     */
    private double math_hypot(float arg0, float arg1) {
        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor3 = hardwareMap.dcMotor.get("motor3");

        initialize();
        if (opModeIsActive()) {

            slamra.start();

            // Put run blocks here.
            while (opModeIsActive()) {

                // Display orientation info.
                globalHeading = getHeading();
                addTelemetryLoopStart();
                Controls();
                telemetry.addData("rot about Z", globalHeading);
                addTelemetryLoopEnd();

// New Camera Stuff
                final int robotRadius = 9; // inches

                TelemetryPacket packet = new TelemetryPacket();
                Canvas field = packet.fieldOverlay();

                T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
                if (up == null) return;

                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = up.pose.getRotation();

                field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
                double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
                double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
                double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
                field.strokeLine(x1, y1, x2, y2);

                telemetry.addData ("X", translation.getX());
                telemetry.addData ("Y", translation.getY());
                telemetry.addData ("<", rotation.getDegrees());
                telemetry.update();

                dashboard.sendTelemetryPacket(packet);
// End new camera stuff

                //telemetry.update();
            }
        }
        slamra.stop();
    }

    /**
     * Describe this function...
     */
    // This costs 7 ms per use.  Use once per loop only if possible.
    private double getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void initialize() {

// New camera stuff
        if (slamra == null) {
            //slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
            slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
        }
// End camera stuff

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // Set motor directions and modes
        initMotors();
        initIMU();
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("Heading (Rot about Z)", globalHeading);
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - getHeading();
        robotError = targetAngle - globalHeading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * Describe this function...
     */
    private void Controls() {

        double DriveSpeed, DriveAngle, Rotate;
        double v0, v1, v2, v3;
        double averageValue;
        double highValue;
        double currentError = 0;

        DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, math_hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)));
        DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;

        DriveAngle = DriveAngle - storedHeading + deltaHeading;


        // overall plan here is to not autocorrect if toggleRotate is true
        //
        // this stupid toggleRotate countup is to catch IMU latency
        if (toggleRotate < 0) {
            toggleRotate++;
            storedHeading = globalHeading;
        }
        if (gamepad1.right_stick_x != 0) {
            toggleRotate = 1;
            storedHeading = globalHeading;
            Rotate = Math.pow(gamepad1.right_stick_x, 3);
        } else {
            Rotate = 0;
            if (toggleRotate == 1) {
                toggleRotate = -10;
                storedHeading = globalHeading;
            }
        }

        if (gamepad1.right_stick_button) {
            deltaHeading = storedHeading;
        }

        // Correct the heading if not currently being controlled
        if (toggleRotate == 0) {
            currentError = getError(storedHeading);
            Rotate = currentError / -15 * (DriveSpeed + 0.2);   // base this on speed?
        }

        DriveSpeed = Math.pow(DriveSpeed, 1);
        v0 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) - Math.sin(DriveAngle / 180 * Math.PI)) + Rotate;
        v1 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) + Math.sin(DriveAngle / 180 * Math.PI)) + Rotate;
        v2 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) + Math.sin(DriveAngle / 180 * Math.PI)) - Rotate;
        v3 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) - Math.sin(DriveAngle / 180 * Math.PI)) - Rotate;
        telemetry.addData("r (magnitude)", DriveSpeed);
        telemetry.addData("robotAngle", DriveAngle);
        telemetry.addData("v0", JavaUtil.formatNumber(v0, 2));
        telemetry.addData("v1", JavaUtil.formatNumber(v1, 2));
        telemetry.addData("v2", JavaUtil.formatNumber(v2, 2));
        telemetry.addData("v3", JavaUtil.formatNumber(v3, 2));
        telemetry.addData("storedHeading", JavaUtil.formatNumber(storedHeading, 2));
        telemetry.addData("deltaHeading", JavaUtil.formatNumber(deltaHeading, 2));
        telemetry.addData("error",JavaUtil.formatNumber(currentError, 2));
//        telemetry.addData("Encoder0", motor0.getCurrentPosition());
//        telemetry.addData("Encoder1", motor2.getCurrentPosition());
//        telemetry.addData("Encoder2", motor1.getCurrentPosition());
//        telemetry.addData("Encoder3", motor3.getCurrentPosition());

        // scale to
        averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v1), Math.abs(v2), Math.abs(v3)));
        averageValue = averageValue / maxSpeed;
        if (averageValue > 1) {
            v0 = v0 / averageValue;
            v1 = v1 / averageValue;
            v2 = v2 / averageValue;
            v3 = v3 / averageValue;
        }

        // scale to no higher than 1
        highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v1), Math.abs(v2), Math.abs(v3), 1));
        motor0.setPower(v0 / highValue);
        motor2.setPower(v1 / highValue);
        motor1.setPower(v2 / highValue);
        motor3.setPower(v3 / highValue);

    }

    /**
     * Describe this function...
     */
    private void initMotors() {
//        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private void initIMU() {
        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
    }

    private void addTelemetryLoopStart() {
        // Other debugging stuff
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", globalHeading);
    }

    private void addTelemetryLoopEnd() {
        // Add anything?
    }

    private double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }
}
