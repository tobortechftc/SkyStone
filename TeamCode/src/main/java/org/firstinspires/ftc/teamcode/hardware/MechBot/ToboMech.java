package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.components.CameraSystem;
import org.firstinspires.ftc.teamcode.components.MechChassis;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.io.File;

public class ToboMech extends Logger<ToboMech> implements Robot2 {

    public enum TargetZone {
        ZONE_A, ZONE_B, ZONE_C, UNKNOWN
    }

    Thread positionThread;
    private Telemetry telemetry;
    public MechChassis chassis;
    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtimeAuto = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public CameraStoneDetector cameraStoneDetector;
    public CameraSystem cameraSystem;
    public File simEventFile;
    public BottomWobbleGoalGrabber bottomWobbleGoalGrabber;
    public TopWobbleGoalGrabber topWobbleGoalGrabber;

    public double auto_chassis_power = .6;
    public double auto_chassis_dist = 100;
    public double auto_chassis_heading = -90;
    public double auto_chassis_power_slow = .2;
    public double auto_chassis_align_power = .22;


    public double auto_rotate_degree = 0;

    private boolean simulation_mode = false;
    public boolean vuforiaTest = false;
    public boolean tensorTest = false;
    public boolean useBottomWobbleGoalGrabber = false;
    public boolean useTopWobbleGoalGrabber = false;

    public void set_simulation_mode(boolean value) {
        simulation_mode = value;
        if (simulation_mode) {
            if (chassis!=null) {
                chassis.set_simulation_mode(value);
            }
        }
    }

    public boolean isSimulationMode() {
        return simulation_mode;
    }

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, ToboMech.AutoTeamColor autoside) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;
        simEventFile = AppUtil.getInstance().getSettingsFile("ToboMech_events.txt"); // at First/settings directory
        // simFile = Paths.get("ToboMech_events.txt");
        this.core = new CoreSystem();
        info("RoboMech configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
        chassis = new MechChassis(core).configureLogging("Mecanum", logLevel); // Log.DEBUG
        if (autoside== ToboMech.AutoTeamColor.DIAGNOSIS) {
            // enable imu for diagnosis
            chassis.enableImuTelemetry(configuration);
        }
        if(tensorTest)
        {
            cameraStoneDetector = new CameraStoneDetector();
            cameraStoneDetector.configure(configuration);
        } else if (vuforiaTest) {
            cameraSystem = new CameraSystem();
            cameraSystem.init(configuration.getHardwareMap());
        }
        chassis.configure(configuration, (autoside!= ToboMech.AutoTeamColor.NOT_AUTO));

        if (simulation_mode) { // need to call after chassis is initialized
            set_simulation_mode(true);
        }

        if(useBottomWobbleGoalGrabber){
            bottomWobbleGoalGrabber = new BottomWobbleGoalGrabber(core);
            bottomWobbleGoalGrabber.configure(configuration, (autoside!= ToboMech.AutoTeamColor.NOT_AUTO));
        }

        if(useTopWobbleGoalGrabber){
            topWobbleGoalGrabber = new TopWobbleGoalGrabber(core);
            topWobbleGoalGrabber.configure(configuration, (autoside!= ToboMech.AutoTeamColor.NOT_AUTO));
        }

        info("ToboMech configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
    }


    @Override
    public void reset(boolean auto) {
        chassis.reset();
        if (auto) {
            chassis.setupTelemetry(telemetry);
        }
    }

    public void end() {
        if (simulation_mode) {
            ReadWriteFile.writeFile(simEventFile, chassis.getSimEvents());
        }
        if (cameraSystem!=null) {
            cameraSystem.end();
        }
    }

    @MenuEntry(label = "TeleOp", group = "Test Chassis")
    public void mainTeleOp(EventManager em) {
        setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        if (chassis!=null && chassis.getGPS()==null) {
            chassis.configureOdometry();
            positionThread = (chassis.getGPS()==null? null: new Thread(chassis.getGPS()));
            if (positionThread!=null)
                positionThread.start();
        }
        chassis.setupTelemetry(telemetry);
        em.onStick(new Events.Listener() { // Left-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY))> 0.2 )
                    return; // avoid conflicting drives
                double right_x = source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY);
                double normalizeRatio = chassis.getMecanumForwardRatio();

                if(!chassis.getNormalizeMode())
                    normalizeRatio = 1;

                // Left joystick for forward/backward and turn
                if (Math.abs(currentY)>0.2) { // car mode
                    chassis.carDrive(currentY*Math.abs(currentY) * normalizeRatio, right_x);
                }else if (Math.abs(currentX) > 0.2) {
                    chassis.turn((currentX > 0 ? 1 : -1), Math.abs(currentX * currentX) * chassis.powerScale()*normalizeRatio);
                } else if (Math.abs(currentY)>0.2) {
                    chassis.yMove((currentY>0?1:-1), Math.abs(currentY * currentY) * chassis.powerScale() * normalizeRatio);
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);

        em.onStick(new Events.Listener() { // Right-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double movingAngle = 0;
                double normalizeRatio = chassis.getMecanumForwardRatio(); // minimum 0.5 when moving forward, and maximum 1.0 when crabbing 90 degree

                if(!chassis.getNormalizeMode())
                    normalizeRatio = 1;

                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY))>0.2 )
                    return; // avoid conflicting drives
                double left_x = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY);
                // right joystick for free crabbing
                if (Math.abs(left_x)>0.1 && Math.abs(currentY)>0.1) {
                    // car drive
                    chassis.carDrive(currentY*Math.abs(currentY)*normalizeRatio, left_x);
                } else if (Math.abs(currentX)+Math.abs(currentY)>0.2) {
                    movingAngle = Math.toDegrees(Math.atan2(currentX, currentY));

                    if(!chassis.getNormalizeMode()) {
                        normalizeRatio = 1;
                    } else if (movingAngle>=-90 && movingAngle<=90) {
                        normalizeRatio = chassis.getMecanumForwardRatio() + (1-chassis.getMecanumForwardRatio()) * (Math.abs(movingAngle)/90.0);
                    } else { // movingAngle is < -90 or > 90
                        normalizeRatio = chassis.getMecanumForwardRatio() + (1-chassis.getMecanumForwardRatio()) * ((180-Math.abs(movingAngle))/90.0);
                    }
                    double lsx = Math.max(Math.abs(currentX), 1.75*chassis.getMinPower()*normalizeRatio)*Math.signum(currentX);
                    double lsy = Math.max(Math.abs(currentY),     chassis.getMinPower()*normalizeRatio)*Math.signum(currentY);
                    double power_lf = (lsy+lsx) * chassis.getFront_ratio() * chassis.getLeft_ratio();
                    double power_lb = (lsy-lsx) * chassis.getBack_ratio() * chassis.getLeft_ratio();
                    double power_rf = (lsy-lsx) * chassis.getFront_ratio() * chassis.getRight_ratio();
                    double power_rb = (lsy+lsx) * chassis.getBack_ratio() * chassis.getRight_ratio();

                    power_lf = Range.clip(power_lf, -1, 1);
                    power_lb = Range.clip(power_lb, -1, 1);
                    power_rf = Range.clip(power_rf, -1, 1);
                    power_rb = Range.clip(power_rb, -1, 1);

                    power_lf *= Math.abs(power_lf)* chassis.powerScale() * normalizeRatio;
                    power_lb *= Math.abs(power_lb)* chassis.powerScale() * normalizeRatio;
                    power_rf *= Math.abs(power_rf)* chassis.powerScale() * normalizeRatio;
                    power_rb *= Math.abs(power_rb)* chassis.powerScale() * normalizeRatio;
                    chassis.freeStyle(power_lf, power_rf, power_lb, power_rb, true);
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null && source.isPressed(Button.BACK) && source.isPressed(Button.START)) {
                    // calibration mode. Only uncomment when testing new motors with chassis wheels suspended
                    chassis.setupEncoders(telemetry);
//                    chassis.freeStyle(1.0, 1.0, 1.0, 1.0, true);
//                    sleep(10000);
//                    chassis.stop();
                } else {
                    chassis.forward(0.3, 30, 3);
                }
            }
        }, new Button[]{Button.DPAD_UP});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    if (source.isPressed(Button.BACK)) {
                        chassis.chassis_test();
                    } else {
                        chassis.forward(0.3, -30, 3);
                    }
                }
            }
        }, new Button[]{Button.DPAD_DOWN});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    chassis.crab(0.45, 30, 3);
                }
            }
        }, new Button[]{Button.DPAD_RIGHT});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    chassis.crab(0.45, -30, 3);
                }
            }
        }, new Button[]{Button.DPAD_LEFT});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    chassis.toggleNormalizeMode();
                } else if(source.getTrigger(Events.Side.LEFT) > 0.3){
                    bottomWobbleGoalGrabber.grabberAuto();
                } else if(source.isPressed(Button.LEFT_BUMPER)){
                    topWobbleGoalGrabber.grabberAuto();
                }
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if(source.getTrigger(Events.Side.LEFT) > 0.3){
                    bottomWobbleGoalGrabber.pivotAuto();
                } else if(source.isPressed(Button.LEFT_BUMPER)){
                    topWobbleGoalGrabber.armAuto();
                }
            }
        }, new Button[]{Button.Y});
    }

    public void setupTelemetryDiagnostics(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Auto ", new Func<String>() {
            @Override
            public String value() {
                return String.format("Mode=%s\n", chassis.getAutoDriveMode().toString());
            }
        });
        line.addData("Test ", new Func<String>() {
            @Override
            public String value() {
                return String.format("Power/auto=%.2f/%.2f, dist=%.0f, rotate_degree=%.1f, tar-x=%.0f,tar-y=%.0f\n",
                        auto_chassis_power,chassis.auto_power, auto_chassis_dist, auto_rotate_degree,
                        chassis.auto_target_x,chassis.auto_target_y);
            }
        });
        line.addData("Result ", new Func<String>() {
            @Override
            public String value() {
                return String.format("dist_err=%.2f, degree_err=%.2f, loop_time=%3.2f, travel_p=%.3f\n",
                        chassis.auto_dist_err,chassis.auto_degree_err, chassis.auto_loop_time, chassis.auto_travel_p);
            }
        });
    }

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        Telemetry.Line line = telemetry.addLine();
        if (chassis==null) return;
        /*
        line.addData(" | Odometry (vl,vr,h) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%5d,%5d,%5d)\n", chassis.verticalLeftEncoder().getCurrentPosition(),
                        chassis.verticalRightEncoder().getCurrentPosition(),chassis.horizontalEncoder().getCurrentPosition());
            }
        });
        line.addData("Odo (x, y, angle) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%4.0f, %4.0f, %4.0f)\n", chassis.odo_x_pos(),chassis.odo_y_pos(),chassis.odo_heading());
            }
        });

        line.addData("Offensive", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isOffensive() ? "Yes" : "No"));
            }
        });
        */
    }


    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }

    public void driveThisPath() throws InterruptedException {
        MechChassis.Point[] points = {new MechChassis.Point(20, 0, 10),
                new MechChassis.Point(40, 0, 20),
                new MechChassis.Point(60, 0, 40),
                new MechChassis.Point(90, 0, 60),
                new MechChassis.Point(100, 20, 65),
                new MechChassis.Point(120, 40, 65),
                new MechChassis.Point(120, -40, 75),
                new MechChassis.Point(100, -20, 75),
                new MechChassis.Point(80, 0, 75),
                new MechChassis.Point(60, 0, -70),
                new MechChassis.Point(40, 0, -70)
        };

        //chassis.set_init_pos(30, 70, 250);
        chassis.driveThrough(.5, points, false, 10);
    }

    public void driveCurve() throws InterruptedException {
        MechChassis.Point[] points = {new MechChassis.Point(50, 50, 45),
                //new MechChassis.Point(100, 100, 90),
                //new MechChassis.Point(80, 140, 90),
                new MechChassis.Point(100, 100, 90)};

        MechChassis.Point[] points2 = {
                new MechChassis.Point(-20, 130, 0),
                new MechChassis.Point(-40, 150, 0),
                new MechChassis.Point(-20, 170, 0),
                new MechChassis.Point(0, 200, 0)};

        chassis.set_init_pos(0, 0, 0);
        //chassis.rotateTo(.5, 20);
        chassis.driveThrough(.5, points, false, 3);
        // chassis.driveThrough(.5, points2, false, 5);
        // chassis.rotateTo(.5, 90);
    }
    public void driveCircle() throws InterruptedException {
        MechChassis.Point[] points = getPointsInCircle(new MechChassis.Point(chassis.odo_x_pos_cm(), chassis.odo_y_pos_cm(), chassis.getCurHeading()),
                new MechChassis.Point(chassis.odo_x_pos_cm(),  chassis.odo_y_pos_cm() + 50, 0), 1, 30);

        // chassis.set_init_pos(20, 90, 0);
        chassis.driveThrough(.6, points, false,2);

    }
    public MechChassis.Point[] getPointsInCircle(MechChassis.Point  start, MechChassis.Point center, int clockwise,  double degreesPerPoint) {
        double radius = Math.hypot(start.x - center.x, start.y - center.y);

        MechChassis.Point[] p = new MechChassis.Point[(int) (360 / degreesPerPoint)];
        for (int i = 0; i < p.length; i++) {
            p[i] = new MechChassis.Point(0, 0, 0);
        }
        double initDegree = Math.atan2(start.x, start.y);
        double degree = initDegree;
        if (clockwise == 1) {
            for (int i = 0;  degree < initDegree + 360; degree= degree + degreesPerPoint) {
                p[i] = new MechChassis.Point (radius * Math.sin(Math.toRadians(degree))+ center.x, radius * Math.cos(Math.toRadians(degree)) + center.y, 0);
                System.out.println(degree);
                if (i != 0) {
                    p[i-1].h =  Math.toDegrees(Math.atan2(p[i].x - p[i-1].x, p[i].y - p[i-1].y));
                }                 //System.out.println(p[i].x + "  " + p[i].y);
                i++;
                System.out.println(i);
            }
        } else {
            for (int i = 0; degree > initDegree - 360; degree = degree - degreesPerPoint) {
                p[i] = new MechChassis.Point(radius * Math.sin(Math.toRadians(degree)) + center.x, radius * Math.cos(Math.toRadians(degree)) + center.y, 0);
                System.out.println(degree);
                if (i != 0) {
                    p[i-1].h =  Math.toDegrees(Math.atan2(p[i].x - p[i-1].x, p[i].y - p[i-1].y));
                }
                //System.out.println(p[i].x + "  " + p[i].y);
                i++;
                System.out.println(i);
            }
        }
        p[p.length - 1] = p[0];
        p[p.length - 2].h = Math.toDegrees(Math.atan2(p[p.length - 1].x - p[p.length - 2].x, p[p.length - 1].y - p[p.length - 2].y));
        return p;
        }


    public void driveAnotherCurve() throws InterruptedException {
        MechChassis.Point[] points = {new MechChassis.Point(120, 35, 90),
                new MechChassis.Point(180, 50, 90),
                new MechChassis.Point(180, 50, 90),
                new MechChassis.Point(200, 75, 90),
                new MechChassis.Point(225, 90, 90),
                new MechChassis.Point(250, 110, 90),
                new MechChassis.Point(200, 135, 90),
                new MechChassis.Point(180, 135, 90),
                new MechChassis.Point(150, 130, 90),
                new MechChassis.Point(130, 110, 90),
                new MechChassis.Point(140, 90, 90),
                new MechChassis.Point(180, 50, 90),
                new MechChassis.Point(240, 35, 90),
                new MechChassis.Point(280, 20, 90),
        };

        chassis.set_init_pos(60, 20, 90);
        chassis.driveThrough(.6, points, false,10);
    }

    @MenuEntry(label = "driveTo/rotateTo", group = "Test Chassis")
    public void testStraight(EventManager em) {
        if (chassis!=null && chassis.getGPS()==null) {
            chassis.configureOdometry();
            chassis.setupTelemetry(telemetry);
            positionThread = (chassis.getGPS()==null? null: new Thread(chassis.getGPS()));
            if (positionThread!=null)
                positionThread.start();
        }
        // chassis.set_init_pos(100,35 ,0);
        if (Thread.interrupted()) return;
        chassis.auto_target_y = chassis.getInit_y_cm();
        chassis.auto_target_x = chassis.getInit_x_cm();
        auto_rotate_degree = auto_chassis_heading = chassis.getInit_heading();

        telemetry.addLine().addData("(BACK) Y/A B/X", "+/- Power(%.2f) Degree(%.0f)", auto_chassis_power,auto_rotate_degree).setRetained(true);
        telemetry.addLine().addData("(L-Tr) Y/A B/X", "+/- Y(%.2f) X(%.0f)", chassis.auto_target_y,chassis.auto_target_x).setRetained(true);
        telemetry.addLine().addData("DPAD-UP/DOWN", "+/- distance(%.2f)", auto_chassis_dist).setRetained(true);
        telemetry.addLine().addData("A:Straight", "B:rotate Y:driveTo").setRetained(true);
        // chassis.setupTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        // chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.START)) {
                    chassis.switchAutoMode();
                } else if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.05;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else if (source.getTrigger(Events.Side.LEFT)>0.5) {
                    chassis.auto_target_y += 10;
                } else if (!source.isPressed(Button.START)) {
                    chassis.driveTo(auto_chassis_power, chassis.auto_target_x,  chassis.auto_target_y, 5);
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.05;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                } else if (source.getTrigger(Events.Side.LEFT)>0.5) {
                    chassis.auto_target_y -= 10;
                } else if (!source.isPressed(Button.START)) {
                    chassis.driveStraight(auto_chassis_power, auto_chassis_dist,  auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 5;
                } else if (source.getTrigger(Events.Side.LEFT)>0.5) {
                    chassis.auto_target_x -= 10;
                } else if (!source.isPressed(Button.START)) {
                    chassis.driveStraight(auto_chassis_power, 1000,  auto_rotate_degree, 3);
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                } else if (source.getTrigger(Events.Side.LEFT)>0.5) {
                    chassis.auto_target_x += 5;
                }  else if (!source.isPressed(Button.START)) {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.B});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveCurve();
                } else {
                    auto_chassis_dist += 10;
                }
            }
        }, new Button[]{Button.DPAD_UP});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveCircle();
                } else {
                    auto_chassis_dist -= 10;
                }
            }
        }, new Button[]{Button.DPAD_DOWN});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveAnotherCurve();
                } else {

                }
            }
        }, new Button[]{Button.DPAD_RIGHT});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveThisPath();
                } else {

                }
            }
        }, new Button[]{Button.DPAD_LEFT});
    }

    @MenuEntry(label = "Auto Rotation", group = "Test Chassis")
    public void testRotationSkyStone(EventManager em) {
        if (chassis!=null && chassis.getGPS()==null) {
            chassis.configureOdometry();
            chassis.setupTelemetry(telemetry);
            positionThread = (chassis.getGPS()==null? null: new Thread(chassis.getGPS()));
            if (positionThread!=null)
                positionThread.start();
        }
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK) X/B", "+/- degree(%.2f)", auto_rotate_degree).setRetained(true);
        chassis.setupTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        // chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                }else {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 10;
                    if (auto_rotate_degree > 150) auto_rotate_degree = 150;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                    if (auto_rotate_degree < -150) auto_rotate_degree = -150;
                }
            }
        }, new Button[]{Button.B});

    }

    @MenuEntry(label = "Tensorflow Test", group = "Test Chassis")
    public void testSkystoneDetection()//loc = 1 left, 2 center, 3 right
    {
        if (cameraStoneDetector!=null) {
            ToboSigma.SkystoneLocation location = cameraStoneDetector.getSkystonePositionTF(true);
        }
    }


    public void deliverFirstWobbleGoal(String targetZone, int startPosition,  String side) throws InterruptedException {

        // start pos - 1 or 2 (1 inside, 2 outside) <---- probably need to change this to enum?
        // still need to change positions to be far left for blue side


        if (targetZone.equals("A")){//0
            chassis.driveTo(.5,  side(45, side), 160, 0, false,  5);

        } else if (targetZone.equals("B")){//1
            chassis.driveTo(.5, side(75, side), 220, 0, false,  5);

        } else if (targetZone.equals("C")){//4
            chassis.driveTo(.5, side(45, side), 280, 0, false,  5);
        } else{
            return;
        }
        // put wobble goal down


    }
    public void doPowerShots( String side) throws InterruptedException {
        chassis.driveTo(.5, side(130, side), 180, 0, false,  5);
        //shoot
        chassis.driveTo(.5, side(150, side), 180, 0, false,  2);
        //shoot
        chassis.driveTo(.5, side(170, side), 180, 0, false,  2);
        //shoot
    }
    public void shootGoal(String side) throws InterruptedException {
        chassis.driveTo(.5, side(90, side), 180, 0, false,  2);
        //shoot

    }
    public void getSecondWobbleGoal(int startPos,  String side) throws InterruptedException { // probably have to change the positions depending on the side
        if(startPos == 1){
            chassis.driveTo(.5, side(120, side), 90, 180, true,  7);
        } else {
            chassis.driveTo(.5, side(60, side), 90, 180, true,  7);
        }
        //grab the wobble goal


    }
    public void deliverSecondWobbleGoal(String targetZone, String side) throws InterruptedException { // we may need to go around the other wobble goal

        // neede to change positions
        if (targetZone.equals("A")){//0
            chassis.driveTo(.5, side(15, side), 200, 0, true,  5);

        } else if (targetZone.equals("B")){//1
            chassis.driveTo(.5, side(105, side), 260, 0, true,  5);

        } else if (targetZone.equals("C")){//4
            chassis.driveTo(.5, side(15, side), 320, 0, true,  5);
        } else{
            return;
        }
        // put the wobble goal down
    }
    public void park() throws InterruptedException {
        chassis.driveTo(.5, chassis.odo_x_pos_cm(), 210, chassis.getCurHeading(), false,  2);
    }
    public double side( double x, String side){
        if ( side.equals("red")){
            return 360 - x;
        }
        return x;
    }

}
