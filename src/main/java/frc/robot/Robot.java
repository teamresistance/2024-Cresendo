package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.test.MecanumDriveCalculator;
import frc.robot.test.motorPID;
import frc.robot.util.Button;
import frc.robot.util.NavX;
import frc.robot.util.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.PIDXController;

public class Robot extends TimedRobot {
    
    private Joystick m_stick;
    private Joystick m_stick2;

    public static final double maxRPM = 5700;

    public static CANSparkMax frontLeftLd = new CANSparkMax(11, MotorType.kBrushless);
    public static CANSparkMax backLeftLd = new CANSparkMax(13, MotorType.kBrushless);
    public static CANSparkMax frontRightLd = new CANSparkMax(15, MotorType.kBrushless);
    public static CANSparkMax backRightLd = new CANSparkMax(17, MotorType.kBrushless);

    public static CANSparkMax frontLeftLg = new CANSparkMax(12, MotorType.kBrushless);
    public static CANSparkMax backLeftLg = new CANSparkMax(14, MotorType.kBrushless);
    public static CANSparkMax frontRightLg = new CANSparkMax(16, MotorType.kBrushless);
    public static CANSparkMax backRightLg = new CANSparkMax(18, MotorType.kBrushless);

    public static motorPID frontLeftLdPID = new motorPID(frontLeftLd, frontLeftLg, false, false, frontLeftLd.getPIDController());
    public static motorPID backLeftLdPID = new motorPID(backLeftLd, backLeftLg, false, false, backLeftLd.getPIDController());
    public static motorPID frontRightLdPID = new motorPID(frontRightLd, frontRightLg, true, false, frontRightLd.getPIDController());
    public static motorPID backRightLdPID = new motorPID(backRightLd, backRightLg, true, false, backRightLd.getPIDController());

    public NavX navX = new NavX(SPI.Port.kMXP);

    public static Button autoBtn = new Button();
    public static Button headingHoldBtn = new Button();
    public static Button resetGyroBtn = new Button();
    public static Button lookAtNote = new Button();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    private PIDController pidControllerX = new PIDController(0.15, 0.000, 0.01);
    private PIDController pidControllerY = new PIDController(5.0, 0.000, 0.00);
    private PIDController pidControllerZ = new PIDController(0.015, 0.00, 0.006);
    private static PIDXController pidHdg = new PIDXController(1.0/80, 0.0, 0.0);     //adj rotSpd for heading

    public boolean auto = false;
    private boolean robotOriented = false;
    private double[] inputs;
    private PhotonCamera camera = new PhotonCamera("Arducam_OV2311_USB_Camera");
    private double yaw, pitch, area2, skew;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,-30.0,20.0));
    PhotonPoseEstimator photonPoseEstimator;

    private AprilTagFieldLayout aprilTagFieldLayout;

    @Override
    public void robotInit() {
        m_stick = new Joystick(0);
        m_stick2 = new Joystick(1);

        autoBtn.setButton(m_stick, 11);
        headingHoldBtn.setButton(m_stick, 12);
        resetGyroBtn.setButton(m_stick, 10);
        lookAtNote.setButton(m_stick, 1);

        frontLeftLdPID.init();
        backLeftLdPID.init();
        frontRightLdPID.init();
        backRightLdPID.init();
        
        pidHdg.setExt(pidHdg, 0.0, 1.0/50, 3.0, 0.05, 0.5, 2.0, true);
        pidHdg.enableContinuousInput(-180, 180);
        try {
            // Your existing constructor code goes here
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // Handle the exception, e.g., print an error message or log it
            System.err.println("Error occurred: " + e.getMessage());
        }
        
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    @Override
    public void teleopPeriodic() {
        frontLeftLdPID.update();
        backLeftLdPID.update();
        frontRightLdPID.update();
        backRightLdPID.update();

        double setPointY = -m_stick.getY();
        double setPointX = -m_stick.getX();
        double setPointZ = m_stick2.getX();

        //Limelight stuff
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //Photonvision stuff
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();


        if (hasTargets){
            // Get information from target.
            yaw = target.getYaw();
            pitch = target.getPitch();
            area2 = target.getArea();
            skew = target.getSkew();
        }
        // Transform2d pose = target.getCameraToTarget();
        // List<TargetCorner> corners = target.getCorners();

        if (autoBtn.isDown() && x != 0.0) auto = true;
        else auto = false;

        if (auto && hasTargets) {
            double pidOutputX = pidControllerX.calculate(-36.0, yaw);
            setPointX -= pidOutputX;
            

            double targetArea = 0.59;
            double pidOutputY = pidControllerY.calculate(targetArea, area2);
            setPointY -= pidOutputY;

            robotOriented = true;
        }
        else{
            robotOriented = false;
        }

        if (lookAtNote.isDown() && x != 0.0){
          
            double pidOutputZ = pidControllerZ.calculate(0.0, x);
            setPointZ = -pidOutputZ;
            robotOriented = true;
            setPointX = 0;
        }
        if (lookAtNote.isUp()){
            robotOriented = false;
        }

        if (headingHoldBtn.isDown()){
          setPointZ = pidHdg.calculateX(navX.getNormalizedTo180(), 0.0);
        }

        if (resetGyroBtn.isDown()){
          navX.reset();
        }

        if (robotOriented) 
        {
            inputs = MecanumDriveCalculator.calculateMecanumDriveRobot(setPointX, setPointY, setPointZ);
        }
        else
        {
            inputs = MecanumDriveCalculator.calculateMecanumDrive(setPointX, setPointY, setPointZ, navX.getAngle());
        }
        frontLeftLdPID.updateSetpoint(inputs[0] * maxRPM);
        frontRightLdPID.updateSetpoint(inputs[1] * maxRPM);
        backLeftLdPID.updateSetpoint(inputs[2] * maxRPM);
        backRightLdPID.updateSetpoint(inputs[3] * maxRPM);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("SetPointX", setPointX);
        SmartDashboard.putNumber("SetPointY", setPointY);
        SmartDashboard.putNumber("SetPointZ", setPointZ);

        SmartDashboard.putNumber("InputFL", inputs[0]);
        SmartDashboard.putNumber("InputFR", inputs[1]);
        SmartDashboard.putNumber("InputBL", inputs[2]);
        SmartDashboard.putNumber("InputBR", inputs[3]);

        //TODO: Fix
        System.out.println(getEstimatedGlobalPose(null));
    }
}
