package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Sensors {
    private final Robot robot;
    private final GoBildaPinpointDriver odometry;

    private double voltage;
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    private int slidesEncoder;
    private double slidesVel;
    public static double slidesInchesPerTick = 35.3 / 1950;

    private int extendoEncoder;
    public static double extendoInchesPerTick = 19.0 / 467;

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    public double[] analogVoltages = new double[analogEncoders.length];

    public Sensors(Robot robot) {
        this.robot = robot;

        odometry = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setOffsets(70, 65);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();

        if (Globals.RUNMODE != RunMode.TELEOP) {
            resetSlidesEncoders();
        }
    }

    public void resetSlidesEncoders() {
        robot.hardwareMap.get(DcMotor.class, "leftRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hardwareMap.get(DcMotor.class, "leftRear").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hardwareMap.get(DcMotor.class, "rightFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hardwareMap.get(DcMotor.class, "rightFront").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        odometry.update();

        slidesEncoder = ((PriorityMotor) robot.hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition();
        slidesVel = ((PriorityMotor) robot.hardwareQueue.getDevice("leftRear")).motor[0].getVelocity();
        extendoEncoder = ((PriorityMotor) robot.hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition();

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }

        updateTelemetry();
    }

    public void resetPosAndIMU() {
        odometry.resetPosAndIMU();
    }

    public void recalibrate() { odometry.recalibrateIMU(); }

    public void setOffsets(double x, double y) {
        odometry.setOffsets(x, y);
    }

    public void setOdometryPosition(double x, double y, double heading) {
        odometry.setPosition(new Pose2d(x, y, heading));
    }

    public void setOdometryPosition(Pose2d pose2d) {
        odometry.setPosition(pose2d);
    }

    public Pose2d getOdometryPosition() {
        Pose2d estimate = odometry.getPosition();
        return new Pose2d(estimate.getX(), estimate.getY(), estimate.getHeading());
    }

    public void setHeading(double heading) {
        odometry.setPosition(new Pose2d(odometry.getPosX(), odometry.getPosY(), heading));
    }

    public double getHeading() {
        return odometry.getHeading();
    }

    public Pose2d getVelocity() {return odometry.getVelocity();}

    public double getSlidesPos() {
        return slidesEncoder * slidesInchesPerTick;
    }

    public double getSlidesVel() {
        return slidesVel * slidesInchesPerTick;
    }

    public double getExtendoPos() {
        return extendoEncoder * extendoInchesPerTick;
    }

    public double getVoltage() {
        return voltage;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("voltage", voltage);

        TelemetryUtil.packet.put("Extendo : position", getExtendoPos());
        TelemetryUtil.packet.put("Extendo : encoder", this.extendoEncoder);
        LogUtil.extendoCurrentPos.set(getExtendoPos());

        TelemetryUtil.packet.put("Slides : position", getSlidesPos());
        TelemetryUtil.packet.put("Slides : encoder", this.slidesEncoder);
        LogUtil.slidesCurrentPos.set(getSlidesPos());

        Pose2d currentPos = getOdometryPosition();
        TelemetryUtil.packet.put("driveCurrentX", currentPos.x);
        TelemetryUtil.packet.put("driveCurrentY", currentPos.y);
        TelemetryUtil.packet.put("driveCurrentAngle (deg)", Math.toDegrees(currentPos.heading));
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPos, getExtendoPos(), "#00ff00");
        LogUtil.driveCurrentX.set(currentPos.x);
        LogUtil.driveCurrentY.set(currentPos.y);
        LogUtil.driveCurrentAngle.set(currentPos.heading);
    }
}
