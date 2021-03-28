package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Field;
import java.util.List;

@TeleOp(name = "LK_FCD_3Odo_Test", group = "")
//@Disabled
public class LK_Field_Centric_Drive_Odox3_Test extends LinearOpMode {

    private BNO055IMU imu;
    /*private DcMotor motor0;
    private DcMotor motor2;
    private DcMotor motor1;
    private DcMotor motor3; */

    private DcMotorEx motor0, motor2, motor1, motor3, odoX, odoY, odoY2;
    private long encoder0, encoder2, encoder1, encoder3, encoderX, encoderY, encoderY2;
    private long encoderX0, encoderY0, encoderY20;

    private static double eTicksPerInch = 82300 / 48;
    private static double eTicksPerRotate = 171500; //171738.8; //170000;

    private double yPos, xPos;
    private double odoHeading, odoHeading0;

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    private double globalHeading;
    private double globalHeading0;
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
        /* motor0 = hardwareMap.dcMotor.get("motor0");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor3 = hardwareMap.dcMotor.get("motor3"); */

        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");  // Configure the robot to use these 4 motor names,
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");  // or change these strings to match your existing Robot Configuration.
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        odoX = hardwareMap.get(DcMotorEx.class, "motor0B");
        odoY = hardwareMap.get(DcMotorEx.class, "motor1B");
        odoY2 = hardwareMap.get(DcMotorEx.class, "motor2B");

        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        initialize();

        // Important Step 3: Option B. Set all Expansion hubs to use the MANUAL Bulk Caching mode
        // Bug info https://github.com/FIRST-Tech-Challenge/SkyStone/issues/232
        /*for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }*/
        for(LynxModule module : allHubs){
            module.clearBulkCache();
            try {
                Class<LynxModule> LynxModuleClass = LynxModule.class;
                Field lynxModuleField = LynxModuleClass.getDeclaredField("lastBulkData");
                lynxModuleField.setAccessible(true);
                lynxModuleField.set(module,module.getBulkData());
            }catch(NoSuchFieldException|IllegalAccessException e){
                e.printStackTrace();
            }
        }

        encoderX0 = odoX.getCurrentPosition();
        encoderY0 = odoY.getCurrentPosition();
        encoderY20 = odoY2.getCurrentPosition();
        globalHeading0 = getHeading();
        odoHeading0 = getOdoHeading();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                // Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
                for (LynxModule module : allHubs) {
                    module.clearBulkCache();
                }

                //encoder0 = motor0.getCurrentPosition();
                //encoder1 = motor1.getCurrentPosition();
                //encoder2 = motor2.getCurrentPosition();
                //encoder3 = motor3.getCurrentPosition();
                encoderX = odoX.getCurrentPosition();
                encoderY = odoY.getCurrentPosition();
                encoderY2 = odoY2.getCurrentPosition();

                // Display orientation info.
                globalHeading = getHeading();
                odoHeading = getOdoHeading();

                updateXY();

                addTelemetryLoopStart();
                Controls();
                telemetry.addData("rot about Z", globalHeading);
                telemetry.addData("odo Heading", odoHeading);
                addTelemetryLoopEnd();

                telemetry.addData ("OdoX", encoderX);
                telemetry.addData ("OdoY", encoderY);
                telemetry.addData ("OdoY2", encoderY2);
                telemetry.addData ("X", xPos);
                telemetry.addData ("Y", yPos);
                telemetry.update();
            }
        }
    }

    private double getOdoHeading() {
        double diffY;

        diffY = encoderY - encoderY2;

        diffY = diffY % eTicksPerRotate;

        diffY = diffY / eTicksPerRotate * 360;

        if (diffY > 180) diffY -= 360;
        if (diffY < -180) diffY += 360;

        return diffY;
    }

    private void updateXY () {
        double deltaEncX, deltaEncY, avgHeading;
        double myHeading;

        deltaEncX = (encoderX - encoderX0) / eTicksPerInch;
        deltaEncY = (encoderY - encoderY0) / eTicksPerInch;


        //myHeading = globalHeading;

        // Future - figure out average heading.  Challenge is the wrap.  Maybe use getError, /2, add to heading???
        myHeading = getAvgHeading(odoHeading0,odoHeading);
        telemetry.addData ("My Average Heading", myHeading);


        yPos = yPos + deltaEncY * Math.cos(Math.toRadians(myHeading));
        xPos = xPos - deltaEncY * Math.sin(Math.toRadians(myHeading));

        yPos = yPos + deltaEncX * Math.sin(Math.toRadians(myHeading));
        xPos = xPos + deltaEncX * Math.cos(Math.toRadians(myHeading));


        encoderX0 = encoderX;
        encoderY0 = encoderY;
        encoderY20 = encoderY2;
        globalHeading0 = globalHeading;
        odoHeading0 = odoHeading;
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

    public double getAvgHeading (double firstHeading, double secondHeading) {
        double robotHeading;

        // find the difference between them
        robotHeading = secondHeading - firstHeading;

        // based on sampling rate, assume large values wrapped
        if (robotHeading > 180) robotHeading -= 360;
        if (robotHeading <= -180) robotHeading += 360;

        robotHeading /= 2;

        robotHeading += firstHeading;

       /* // make them positive angles first
        if (firstHeading < 0) firstHeading += 360;
        if (secondHeading < 0) secondHeading += 360;

        robotHeading = (secondHeading + firstHeading) / 2; */

        // then recalculate to -179 to +180 range
        while (robotHeading > 180)  robotHeading -= 360;
        while (robotHeading <= -180) robotHeading += 360;

        return robotHeading;
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
        telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
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
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        odoX.setDirection(DcMotorSimple.Direction.FORWARD);
        odoY.setDirection(DcMotorSimple.Direction.REVERSE);
        odoY2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoY2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
