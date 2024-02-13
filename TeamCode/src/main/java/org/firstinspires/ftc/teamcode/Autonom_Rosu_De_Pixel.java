package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var_Red.CV_detectionType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Autonom_Rosu_De_Pixel extends LinearOpMode {
    double rectx, recty, hperw,x;
    boolean outOfThread = false;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouRosu pipelineRosu = new PachetelNouRosu();
    public ChestiiDeAutonom c = new ChestiiDeAutonom();
    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        CV_detectionType = Var_Red.DetectionTypes.DAY;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam.setPipeline(pipelineRosu);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Var_Red.Webcam_w, Var_Red.Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        while (!isStopRequested() && !isStarted()) {
            try {
                rectx = pipelineRosu.getRect().width;
                recty = pipelineRosu.getRect().height;
                hperw = recty / rectx;
                telemetry.addData("rectangle width:", rectx);
                telemetry.addData("rectangle height:", recty);
                telemetry.addData("height / width:", hperw);
                x = pipelineRosu.getRect().x + pipelineRosu.getRect().width/2.0;
                telemetry.addData("x:",pipelineRosu.getRect().x + pipelineRosu.getRect().width/2);
                if (x < 350 && x > 100) {
                    varrez = 2;
                }
                else if(x > 350){
                    varrez = 3;
                }
                else{
                    varrez = 1;
                }
                telemetry.addData("caz:", varrez);
            }
            catch (Exception E) {
                varrez = 1;
                telemetry.addData("Webcam error:", "please restart");
                telemetry.update();
            }
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.783464, -62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(16, -38),Math.toRadians(45)))
                .build();
        if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(14, -33))
                    .build();
        }
        else if(varrez == 1){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(15,-48),Math.toRadians(130)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(10,-34),Math.toRadians(160)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        Pus_pe_tabla.start();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(53, -29),Math.toRadians(180)))
                .build();
        if(varrez == 3){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(16,-48),Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-41),Math.toRadians(180)))
                    .build();
        }
        else if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-34),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
//        c.melctarget(0.8,1300,3000);
//        c.target(-800,1300,c.getSlider(),3000,10);
//        c.kdf(200);
//        c.target(-100,1300,c.getSlider(),3000,10);
        if(varrez == 2){
            c.kdf(1100);
        }
        else if(varrez == 1){
            c.kdf(2100);
        }
        else{
            c.kdf(500);
        }
        c.deschidere();
        c.kdf(600);
        Brat_jos.start();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(33,-10),Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(-43,-10),Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(ts);
        c.inchidere();
        c.pixel_retreat();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(30,-10))
                .build();
        drive.followTrajectorySequence(ts);
        Pus_pe_tabla.start();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(52.5,-34))
                .build();
        if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(52.5,-30))
                    .build();
        }
        else if(varrez == 1){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(52.5,-41))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        c.kdf(1000);
        c.deschidere();
        c.kdf(500);
        c.melctarget(2.3,3000,3000);
    }
    private final Thread Pus_pe_tabla = new Thread(new Runnable() {
        @Override
        public void run() {
            outOfThread = false;
            c.melctarget(0.85,1300,3000);
            c.target(-800,1300,c.getSlider(),3000,10);
            c.kdf(200);
            c.target(-300,1300,c.getSlider(),3000,10);
            c.kdf(500);
            outOfThread = true;
        }
    });
    private final Thread Brat_jos = new Thread(new Runnable() {
        @Override
        public void run() {
            c.kdf(500);
            c.melctarget(2.3,1300,3000);
        }
    });
}
