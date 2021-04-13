package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.lang.reflect.Field;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.time.format.DateTimeFormatter;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

@TeleOp(name = "3Odo_Test_4_T265_log", group = "")
//@Disabled
public class Odo_Test_4_T265_log extends LinearOpMode {

    // New Camera Stuff
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //LK add from instructions
    //Transform2d cameraToRobot = new Transform2d();
    Transform2d cameraToRobot = new Transform2d(new Translation2d(-8 * 0.0254, 0.0 * 0.254), new Rotation2d());

    double encoderMeasurementCovariance = 0.1; //0.8;
    //Pose2d startingPose = new Pose2d(-63 * 0.0254, 57 * 0.0254, new Rotation2d());
// End New Camera Stuff

    private BNO055IMU imu;
    private Writer fileWriter;

    private DcMotorEx motor0, motor2, motor1, motor3, odoXL, odoY, odoXR;
    private long encoder0, encoder2, encoder1, encoder3, encoderY, encoderXL, encoderXR;
    private long encoderY0, encoderXL0, encoderXR0;

    private static double eTicksPerInch = 82300 / 48;
    private static double eTicksPerRotate = 171500; //171738.8; //170000;

    private double yPos, xPos;
    private double odoHeading, odoHeading0;

    private double robotX, robotY, robotH;

    private double t265X, t265Y, t265Rot;

    private double targetX, targetY, targetRot;
    private int navigate = 0;

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    private double globalHeading;
    private double globalHeading0;
    private double storedHeading = 0;
    private double deltaHeading = 0;
    private int toggleRotate;

    private final double maxSpeed = 1;//0.2;

    double DriveSpeed, DriveAngle, Rotate;
    double v0, v1, v2, v3;
    double averageValue;
    double highValue;
    double currentError = 0;

    boolean accurate = true;
    int navStep = 0;
    private ElapsedTime navTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime logTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int lastStep = 0;

    // type, x, y, rot
    // where type 1 = accurate, 2 = transition, 3 = pause..., 4 = (not used yet), 999 = end routine
    double[][] autoScript = {
/*            {1, 0, 0, 0},
//            {2, 10, -10, 90},
//            {2, 20, 0, -90},
//            {1, 36, 0, -120},
            {1, 24, -20, -90},
            {3, 500},
            {2, 24, 0, 90},
            {2, 48, 0, 90},
            {1, 48, 24, 90},
            {3, 1000},
            {2, 48, 4, 90},
            {1, 24, 0, 0},
            {3, 1000},
            {1, 0, 0, 0},
            {999}*/
            {2, 0, 0, 0},
            {2, 51, 0, 0},
            {1, 60, -40, 0},
            {3, 1000},
            {1, 60, -48, 0},
            {3, 1000},
            {1, 60, -56, 0},
            {3, 1000},
            {1, 94, -23, 90},
            {3, 2000},
            {2, 62, -46, 180},
            {1, 19, -48, 180},
            {3, 2000},
            {2, 60, -35, 180},
            {1, 84, -27, 90},
            {3, 2000},
            {1, 74, -26, 90},
            {999}
    };

    // Hypotenuse function I used in Blocky?
    private double math_hypot(float arg0, float arg1) {
        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
    }

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");  // Configure the robot to use these 4 motor names,
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");  // or change these strings to match your existing Robot Configuration.
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        odoY = hardwareMap.get(DcMotorEx.class, "motor0B");
        odoXR = hardwareMap.get(DcMotorEx.class, "motor1B");
        odoXL = hardwareMap.get(DcMotorEx.class, "motor2B");

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

        encoderY0 = odoY.getCurrentPosition();
        encoderXL0 = odoXL.getCurrentPosition();
        encoderXR0 = odoXR.getCurrentPosition();
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
                encoderY = odoY.getCurrentPosition();
                encoderXL = odoXL.getCurrentPosition();
                encoderXR = odoXR.getCurrentPosition();

                // Display orientation info.
                globalHeading = getHeading();
                odoHeading = getOdoHeading();

                updateXY();
                cameraStuff();

                updateLog();

                // At this point, we have two different sources of odometry data available.
                // xPos, yPos, globalHeading (and odoHeading!)
                // -or-
                // t265X, t265Y, t265Rot
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                robotX=t265X;
                robotY=t265Y;
                //robotH=t265Rot;
                globalHeading=t265Rot;

                addTelemetryLoopStart();
                Controls();

                if (navigate==1) {
                    autoDrive();
                }
                else if (navigate==2) {
                    scriptedNav();
                }
                else {
                    userDrive();
                }


                // set motor power
                motor0.setPower(v0);
                motor2.setPower(v1);
                motor1.setPower(v2);
                motor3.setPower(v3);

                addTelemetryLoopEnd();

                telemetry.update();
            }
        }

        try {
            fileWriter.close();
        } catch (IOException e) {
            throw new RuntimeException("Cannot close file", e);
        }

        slamra.stop();
    }

    private void writeToFile(String data) {
        try {
            fileWriter.write(data);
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }

    private void updateLog() {
        if (logTime.milliseconds()>100) {
            logTime.reset();
            writeToFile(String.valueOf(elapsedTime.milliseconds()));
            writeToFile(", ");
            writeToFile(String.valueOf(xPos));
            writeToFile(", ");
            writeToFile(String.valueOf(yPos));
            writeToFile(", ");
            writeToFile(String.valueOf(globalHeading));
            writeToFile(", ");
            writeToFile(String.valueOf(odoHeading));
            writeToFile(", ");
            writeToFile(String.valueOf(t265X));
            writeToFile(", ");
            writeToFile(String.valueOf(t265Y));
            writeToFile(", ");
            writeToFile(String.valueOf(t265Rot));
            writeToFile("\n");
        }
    }

    // T265 routine
    private void cameraStuff() {
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
        //telemetry.update();

        dashboard.sendTelemetryPacket(packet);

        t265X=translation.getX();
        t265Y=translation.getY();
        t265Rot=rotation.getDegrees();
        // End new camera stuff
    }

    // Simple state machine for scripted navigation actions
    private void scriptedNav() {

        // reset the timer each new step (could be used to time out actions;
        // is used for delays)
        if (navStep != lastStep) {
            navTime.reset();
            lastStep=navStep;
        }

        double action=autoScript[navStep][0];

        telemetry.addData("NavStep", navStep);
        //telemetry.addData("Action", autoScript[navStep]);

        // accurate position
        if (action==1) {
            targetX=autoScript[navStep][1];
            targetY=autoScript[navStep][2];
            targetRot=autoScript[navStep][3];
            accurate=true;
            autoDrive();
        }
        // transitional position
        else if (action==2) {
            targetX=autoScript[navStep][1];
            targetY=autoScript[navStep][2];
            targetRot=autoScript[navStep][3];
            accurate=false;
            autoDrive();
        }
        // delay
        else if (action==3) {
            if (navTime.milliseconds()>=autoScript[navStep][1]) navStep++;
        }
        // end the navigation
        else if (action==999) {
            navigate=0;
        }
    }

    // get heading from the odometry; accuracy varies :-(
    private double getOdoHeading() {
        double diffX;
        diffX = encoderXR - encoderXL;
        diffX = diffX % eTicksPerRotate;
        diffX = diffX / eTicksPerRotate * 360;
        if (diffX > 180) diffX -= 360;
        if (diffX < -180) diffX += 360;
        return diffX;
    }

    // Get XY position data from odometry wheels
    private void updateXY () {
        double deltaEncX, deltaEncY, avgHeading;
        double myHeading;

        //deltaEncX = (encoderXL - encoderXL0) / eTicksPerInch;
        deltaEncX = (encoderXL + encoderXR - encoderXL0 - encoderXR0) / 2.0 / eTicksPerInch;
        deltaEncY = (encoderY - encoderY0) / eTicksPerInch;

        //myHeading = globalHeading;
        // Future - figure out average heading.  Challenge is the wrap.  Maybe use getError, /2, add to heading???
        //myHeading = getAvgHeading(odoHeading0,odoHeading);
        myHeading = getAvgHeading(globalHeading0,globalHeading);

        telemetry.addData ("My Average Heading", myHeading);

        xPos = xPos + deltaEncX * Math.cos(Math.toRadians(myHeading));
        yPos = yPos + deltaEncX * Math.sin(Math.toRadians(myHeading));

        xPos = xPos + deltaEncY * Math.sin(Math.toRadians(myHeading));
        yPos = yPos - deltaEncY * Math.cos(Math.toRadians(myHeading));

        encoderXL0 = encoderXL;
        encoderY0 = encoderY;
        encoderXR0 = encoderXR;
        globalHeading0 = globalHeading;
        odoHeading0 = odoHeading;
    }

    // Get heading data from IMU
    // This costs 7 ms per use.  Use once per loop only if possible.
    private double getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Initialization routine
    private void initialize() {
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

// New camera stuff
        if (slamra == null) {
            //slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
            //slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
        }
// End camera stuff

        DateFormat df = new SimpleDateFormat("yyyyMMdd-HHmmss");
        String dateString = df.format(new Date());

        String logFilePath = String.format("%s/FIRST/odo4_t265_%s.txt", Environment.getExternalStorageDirectory().getAbsolutePath(), dateString );
        try {
            FileWriter writer = new FileWriter(logFilePath);
            fileWriter = new BufferedWriter(writer);
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }

        // Set motor directions and modes
        initMotors();
        initIMU();

        slamra.start();

        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("Heading (Rot about Z)", globalHeading);
            cameraStuff();
            telemetry.update();
        }
    }

    // average of two headings
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


    // Get heading error
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - getHeading();
        robotError = targetAngle - globalHeading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    // Determine motor speeds when under driver control
    private void userDrive () {
        DriveSpeed = Math.pow(DriveSpeed, 1);
        v0 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) - Math.sin(DriveAngle / 180 * Math.PI)) + Rotate;
        v1 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) + Math.sin(DriveAngle / 180 * Math.PI)) + Rotate;
        v2 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) + Math.sin(DriveAngle / 180 * Math.PI)) - Rotate;
        v3 = DriveSpeed * (Math.cos(DriveAngle / 180 * Math.PI) - Math.sin(DriveAngle / 180 * Math.PI)) - Rotate;

        // scale so average motor speed is not more than maxSpeed
        averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v1), Math.abs(v2), Math.abs(v3)));
        averageValue = averageValue / maxSpeed;
        if (averageValue > 1) {
            v0 /= averageValue;
            v1 /= averageValue;
            v2 /= averageValue;
            v3 /= averageValue;
        }

        // scale to no higher than 1
        highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v1), Math.abs(v2), Math.abs(v3), 1));
        v0 /= highValue;
        v1 /= highValue;
        v2 /= highValue;
        v3 /= highValue;
    }

    // Determine motor speeds when under automatic control
    private void autoDrive () {
        double distance, deltaX, deltaY, deltaRot, pDist, pRot, navAngle;
        v0=0;
        v1=0;
        v2=0;
        v3=0;
        deltaX = targetX-robotX;  // error in x
        deltaY = targetY-robotY;  // error in y
        telemetry.addData("DeltaX", JavaUtil.formatNumber(deltaX, 2));
        telemetry.addData("DeltaY", JavaUtil.formatNumber(deltaY, 2));
        deltaRot = getError(targetRot);  // error in rotation
        distance = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));  // distance (error) from xy destination

        //exit criteria if destination has been adequately reached
        if (accurate==false && distance<2) {  // no rotation component here
            navStep++;
            return;
        }
        if (distance<0.5 && Math.abs(deltaRot)<0.2) {
            navStep++;
            return;
        }

        navAngle = Math.toDegrees(Math.atan2(deltaY,deltaX));  // angle to xy destination (vector when combined with distance)
        // linear proportional at 12", minimum 0.025
        pDist = Math.max(Math.min(distance/12,1),0.025);
        if (accurate==false) pDist = 1;  // don't bother with proportional when hitting transitional destinations
        // linear proportional at 15°, minimum 0.025
        pRot = Math.max(Math.min(Math.abs(deltaRot)/15,1),0.025)*Math.signum(deltaRot)*-1;

        telemetry.addData("NavDistance", JavaUtil.formatNumber(distance, 2));
        telemetry.addData("NavAngle", JavaUtil.formatNumber(navAngle, 2));
        telemetry.addData("NavRotation", JavaUtil.formatNumber(deltaRot, 2));
        telemetry.addData("pDist", JavaUtil.formatNumber(pDist, 2));
        telemetry.addData("pRot", JavaUtil.formatNumber(pRot, 2));

        navAngle -= globalHeading;  // need to account for how the robot is oriented
        DriveSpeed = pDist * 1;  // 1 here is maxspeed; could be turned into a variable
        // the following adds the mecanum X, Y, and rotation motion components for each wheel
        v0 = DriveSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + pRot;
        v1 = DriveSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + pRot;
        v2 = DriveSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - pRot;
        v3 = DriveSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - pRot;

        // scale to no higher than 1
        highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v1), Math.abs(v2), Math.abs(v3), 1));
        v0 /= highValue;
        v1 /= highValue;
        v2 /= highValue;
        v3 /= highValue;
    }

    // Interpret user control inputs
    private void Controls() {

        // This blob is for manually entering desitnations
        if (gamepad2.a) {
            targetX = Math.round(xPos);
            targetY = Math.round(yPos);
            targetRot = Math.round(globalHeading);
        }
        if (gamepad2.dpad_up) targetX++;
        if (gamepad2.dpad_down) targetX--;
        if (gamepad2.dpad_left) targetY++;
        if (gamepad2.dpad_right) targetY--;
        if (gamepad2.x) targetRot++;
        if (gamepad2.y) targetRot--;
        if (gamepad2.start) navigate=1;

        // Cancels auto navigation
        if (gamepad2.b) {
            navigate=0;
            storedHeading = globalHeading;
        }

        // Set to home position
        if (gamepad2.right_bumper) {
            targetX=0;
            targetY=0;
            targetRot=0;
        }

        // Begin scripted navigation
        if (gamepad2.left_bumper) {
            navigate=2;
            navStep=0;
            navTime.reset();
        }

        // Get speed and direction from left stick
        DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, math_hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)));
        DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
        // Adjust heading for corrections
        DriveAngle = DriveAngle - storedHeading + deltaHeading;

        // overall plan here is to not autocorrect if toggleRotate is true
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

        // Store heading correction
        if (gamepad1.right_stick_button) {
            deltaHeading = storedHeading;
        }

        // Correct the heading if not currently being controlled
        if (toggleRotate == 0) {
            currentError = getError(storedHeading);
            Rotate = currentError / -15 * (DriveSpeed + 0.2);   // base this on speed?
        }

    }

    // Motor init
    private void initMotors() {
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        odoY.setDirection(DcMotorSimple.Direction.FORWARD);
        odoXL.setDirection(DcMotorSimple.Direction.REVERSE);
        odoXR.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoXL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoXR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // IMU init
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
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", globalHeading);
    }

    private void addTelemetryLoopEnd() {
        telemetry.addData("r (magnitude)", DriveSpeed);
        telemetry.addData("robotAngle", DriveAngle);
        telemetry.addData("storedHeading", JavaUtil.formatNumber(storedHeading, 2));
        telemetry.addData("deltaHeading", JavaUtil.formatNumber(deltaHeading, 2));
        telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
        telemetry.addData("v0", JavaUtil.formatNumber(v0, 2));
        telemetry.addData("v1", JavaUtil.formatNumber(v1, 2));
        telemetry.addData("v2", JavaUtil.formatNumber(v2, 2));
        telemetry.addData("v3", JavaUtil.formatNumber(v3, 2));
        telemetry.addData("rot about Z", globalHeading);
        telemetry.addData("odo Heading", JavaUtil.formatNumber(odoHeading, 2));
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Target Rot", targetRot);
        telemetry.addData ("OdoY", encoderY);
        telemetry.addData ("OdoXL", encoderXL);
        telemetry.addData ("OdoXR", encoderXR);
        telemetry.addData ("X", JavaUtil.formatNumber(xPos, 2));
        telemetry.addData ("Y", JavaUtil.formatNumber(yPos, 2));
    }

    // Calculate loop time for performance optimization
    private double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }
}
