package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //LK add from instructions
    Transform2d cameraToRobot = new Transform2d();
    double encoderMeasurementCovariance = 0.1; //0.8;
    Pose2d startingPose = new Pose2d(-63 * 0.0254, 57 * 0.0254, new Rotation2d());

    @Override
    public void init() {
        if (slamra == null) {
            //slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
            slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
        }
        //slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
/*
// This is the transformation between the center of the camera and the center of the robot
        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
// Set to the starting pose of the robot
        Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

        T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
 */

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
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
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}
