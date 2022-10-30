package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;

//@Disabled
@Autonomous(name="detecting tag")
public class detectApriltag extends LinearOpMode {

    DcMotorEx A;
    DcMotorEx B;
    DcMotorEx C;
    DcMotorEx D;

    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // for logitech webcam
    double fx = 821.993;
    double fy = 821.993;
    double cx = 330.489;
    double cy = 248.997;

    // in meters
    double tagsize = 0.06; // 60mm tag size

    // apriltags
    int LEFT = 0;
    int MIDDLE = 2;
    int RIGHT = 5;
    int tagNumber = 10;
    boolean tagFound = false;

    AprilTagDetection tagOfInterest = null;

    public void drive(double power, double xCoord, double yCoord, double hValue, int numSec) {

        // xCoord (left stick x-axis)
        // yCoord (left stick y-axis)
        // hValue (right stick x-axis)

        if (opModeIsActive()) {
            double y = -yCoord;
            double x = -xCoord;
            double h = -hValue * 0.6;
            double newX = (x * 0.707) - (y * 0.707);
            double newY = (x * 0.707) + (y * 0.707);
            double c = 0.5;

            int aTarget = (int) (A.getCurrentPosition() + (newX + h) * c);
            int bTarget = (int) (B.getCurrentPosition() + (-newY - h) * c);
            int cTarget = (int) (C.getCurrentPosition() + (newX - h) * c);
            int dTarget = (int) (D.getCurrentPosition() + (-newY + h) * c);

            A.setTargetPosition(aTarget * numSec);
            B.setTargetPosition(bTarget * numSec);
            C.setTargetPosition(cTarget * numSec);
            D.setTargetPosition(dTarget * numSec);

            A.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            C.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            D.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            A.setPower(power);
            B.setPower(power);
            C.setPower(power);
            D.setPower(power);

            while ((A.isBusy() || B.isBusy() || C.isBusy() || D.isBusy())) {
                // wait for code to finish
            }

            A.setPower(0);
            B.setPower(0);
            C.setPower(0);
            D.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // without monitor view:
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

        A = hardwareMap.get(DcMotorEx.class, "motorA");
        B = hardwareMap.get(DcMotorEx.class, "motorB");
        C = hardwareMap.get(DcMotorEx.class, "motorC");
        D = hardwareMap.get(DcMotorEx.class, "motorD");

        A.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        C.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        D.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        A.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        C.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        D.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        B.setDirection(DcMotor.Direction.REVERSE);
        C.setDirection(DcMotor.Direction.REVERSE);

        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {

                // labeling which tag is detected
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagNumber = tag.id;
                        break;
                    }

                }
            }
        }

        waitForStart();

        while (opModeIsActive()) {

            if (!tagFound) {
                // didn't see tag
                telemetry.addLine("No tag has been found");
                telemetry.update();
            } else {
                if (tagNumber == 0) {
                    telemetry.addLine("Tag 0 has been found... going LEFT");
                    telemetry.addLine("Tag snapshot:\n");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();

                    // test
                    drive(0.5, 1, 1, 0, 5);

                } else if (tagNumber == 5) {
                    telemetry.addLine("Tag 5 has been found... going FORWARD");
                    telemetry.addLine("Tag snapshot:\n");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();

                    drive(0.5, -1, -1, 0, 5);

                } else if (tagNumber == 2){
                    telemetry.addLine("Tag 2 has been found... going RIGHT");
                    telemetry.addLine("Tag snapshot:\n");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();

                    drive(0.5, 0, 1, 0, 5);
                }

                else {
                    telemetry.addLine("Detected wrong tag, not going anywhere");
                    telemetry.addLine("Tag snapshot:\n");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();
                }
            }
        }
    }
        void tagToTelemetry (AprilTagDetection detection) {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }
