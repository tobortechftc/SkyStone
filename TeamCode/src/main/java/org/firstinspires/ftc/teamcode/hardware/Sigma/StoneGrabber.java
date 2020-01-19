package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.CombinedOrientationSensor;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import static java.lang.Thread.*;

/**
 * StoneGrabber spec:
 */
public class StoneGrabber extends Logger<StoneGrabber> implements Configurable {

    final private CoreSystem core;

    private DcMotor lifter;
    private AdjustableServo arm;
    private AdjustableServo wrist;
    private AdjustableServo grabber;
    private AdjustableServo capstoneServo;
    private ElapsedTime SGTimer = new ElapsedTime();
    private double waitSec;


    private final double ARM_UP = 0.06;
    private final double ARM_READY_GRAB = 0.96;
    private final double ARM_DOWN = 0.86;  // right position to grab stone inside
    private final double ARM_DOWN_MORE = ARM_DOWN+0.06;  // right position to grab stone inside
    private final double ARM_DOWN_MORE_CAP = ARM_DOWN+0.09;  // right position to grab stone inside
    private final double ARM_DOWN_SAFE = 0.86;
    private final double ARM_INITIAL = 0.82;
    private final double ARM_OUT_INIT = 0.45;
    private final double ARM_IN = 0.63;
    private final double ARM_LOW = 0.56;
    private final double ARM_OUT = 0.33;
    private final double ARM_OUT_MORE = 0.25;
    private final double ARM_OUT_AUTO = 0.41;
    private final double ARM_DOWN_FOR_CAP = 0.74;
    private final double ARM_CAPSTONE = 0.76;
    private final double ARM_CAPSTONE_MORE = 0.87;
    private final double ARM_DELIVER = 0.26;
    private final double ARM_DELIVER_HIGHER = 0.22;
    private final double ARM_DELIVER_THROW = 0.12;
    private final double ARM_MIN = 0.1;
    private final double ARM_INC_UNIT = 0.02;

    private final double WRIST_PARALLEL = 0.57;
    private final double WRIST_PERPENDICULAR = 0.001;
    private final double WRIST_INIT = WRIST_PERPENDICULAR;
    private final double WRIST_INC_UNIT = 0.01;
    private final double WRIST_CAPSTONE = WRIST_PARALLEL - 0.01;

    private final double GRABBER_INIT = 0.25;
    private final double GRABBER_OPEN_IN = 0.5;
    private final double GRABBER_OPEN = 0.86;
    private final double GRABBER_CLOSE = 0.25;

    //private final int LIFT_RUN_TO_POSITION_OFFSET = 20;   // V5.2 
    private final int LIFT_RUN_TO_POSITION_OFFSET = 150;  // V5.3, new control for goBilda 5205 motor
    private final int LIFT_DOWN_GRAB = 20;
    private final int LIFT_DOWN = 20;
    private final int LIFT_GRAB_READY_CAPSTONE = 440;
    private final int LIFT_GRAB = 600;
    private final int LIFT_GRAB_AUTO = 640;
    private final int LIFT_MIN = 0;
    private final int LIFT_MAX = 4800;
    private final int LIFT_SAFE_SWING_AUTO = 1000;
    private final int LIFT_SAFE_BRIDGE = 1086;
    private final int LIFT_SAFE_SWING_IN = 1200;
    private final int LIFT_SAFE_DELIVERY = 800;
    private final int LIFT_SAFE_SWING = 1000;
    private final int LIFT_UP_FOR_REGRAB = 430;
    private final int LIFT_UP_FOR_CAP = 1300;
    private final int LIFT_UP_BEFORE_CAP = 1200;
    private final int LIFT_UP_FINAL_CAP = 2100;
    //private final double LIFT_POWER = 0.5;   // V5.2
    private final double LIFT_POWER = 1.0;  // V5.3
    private final double LIFT_POWER_SLOW = 0.5;
    private final double LIFT_POWER_SLOW_DOWN = 0.3;
    private final int LIFT_DELIVER = 1000;

    private final double CAPSTONE_INIT = 0.0;
    private final double CAPSTONE_OUT = 0.6;



    private boolean armIsDown = false;
    private boolean armIsIn = true;
    private boolean isGrabberOpened = false;
    private boolean isWristParallel = false;
    private boolean isCapstoneServoOut = false;
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public String getUniqueName() {
        return "StoneGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public StoneGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto, boolean armOut) {
        if (arm != null)
            armInit(armOut);
        if (wrist!=null) {
            if (armOut)
                wristParallel();
            else
                wristInit();
        }
        if (grabber!=null)
            grabberInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        arm = new AdjustableServo(0,1).configureLogging(
                logTag + ":arm", logLevel
        );
        arm.configure(configuration.getHardwareMap(), "arm");
        configuration.register(arm);
        // armInit(false);

        wrist = new AdjustableServo(0,1).configureLogging(
                logTag + ":wrist", logLevel
        );
        wrist.configure(configuration.getHardwareMap(), "wrist");
        configuration.register(wrist);
        // wristInit();

        grabber = new AdjustableServo(0,1).configureLogging(
                logTag + ":grabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);
        grabberInit();

        capstoneServo = new AdjustableServo(0,1).configureLogging(
                logTag + ":capstoneServo", logLevel
        );
        capstoneServo.configure(configuration.getHardwareMap(), "capstoneServo");
        configuration.register(capstoneServo);
        capstoneServoInit();

        lifter = configuration.getHardwareMap().tryGet(DcMotor.class, "lifter");
        //if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // register hanging as configurable component
        // configuration.register(this);
    }

    public boolean isArmInside() {
        return armIsIn;
    }

    public boolean isArmDown() {
        return armIsDown;
    }

    public void armInit(boolean armOut) {
        arm.setPosition((armOut?ARM_OUT_INIT:ARM_INITIAL));
        armIsDown = false;
        armIsIn = !armOut;
    }

    public void armOut() {
        arm.setPosition(ARM_OUT);
        armIsDown = false;
        armIsIn = false;
    }

    public void armDownInc() {
        double cur_pos = arm.getPosition();
        cur_pos += ARM_INC_UNIT;
        if (cur_pos>1) cur_pos=1;
        arm.setPosition(cur_pos);
        armIsDown = false;
        if (cur_pos>ARM_IN)
            armIsIn = true;
        else
            armIsIn = false;
        if (Math.abs(cur_pos-ARM_DOWN)<0.2)
            armIsDown = true;
    }

    public void armUpInc(boolean force) {
        double cur_pos = arm.getPosition();
        cur_pos -= ARM_INC_UNIT;
        if ((cur_pos<ARM_MIN) && !force)
            cur_pos=ARM_MIN;
        if (cur_pos<0) cur_pos=0;
        arm.setPosition(cur_pos);
        armIsDown = false;
        if (cur_pos<ARM_LOW)
            armIsIn = false;
        else
            armIsIn = true;
        if (Math.abs(cur_pos-ARM_DOWN)<0.2)
            armIsDown = true;
    }

    public void armUp() {
        arm.setPosition(ARM_UP);
        armIsDown = false;
        armIsIn = false;
    }

    public void armDown() {
        arm.setPosition(ARM_DOWN);
        armIsDown = true;
        armIsIn = true;
    }

    public void armDownSafe() {
        arm.setPosition(ARM_DOWN_SAFE);
        armIsDown = true;
        armIsIn = true;
    }

    public void armDeliver() {
        arm.setPosition(ARM_DELIVER);
        armIsDown = false;
        armIsIn = false;
    }

    public void armAuto() {
        if (armIsDown) {
            armUp();
        } else {
            armDown();
        }
    }

    public void wristInit() {
        if (wrist==null) return;
        wrist.setPosition(WRIST_INIT);
        isWristParallel = false;
    }

    public void wristParallel () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PARALLEL);
        isWristParallel = true;
    }

    public void wristPerpendicular () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PERPENDICULAR);
        isWristParallel = false;
    }

    public void wristAuto() {
        if (isWristParallel) wristPerpendicular();
        else wristParallel();
    }

    public void wristLeftInc() {
        double cur_pos = wrist.getPosition();
        cur_pos -= WRIST_INC_UNIT;
        if (cur_pos<0) cur_pos=0;
        wrist.setPosition(cur_pos);
        if (cur_pos<=(WRIST_PARALLEL+0.2))
            isWristParallel = true;
        else
            isWristParallel = false;

    }
    public void wristRightInc() {
        double cur_pos = wrist.getPosition();
        cur_pos += WRIST_INC_UNIT;
        if (cur_pos>1) cur_pos=1;
        wrist.setPosition(cur_pos);
        if (cur_pos>=(WRIST_PERPENDICULAR-0.2))
            isWristParallel = false;
        else
            isWristParallel = true;

    }

    public Progress moveWrist(boolean parallel) {
        double target = parallel ? WRIST_PARALLEL : WRIST_PERPENDICULAR;
        isWristParallel = parallel;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveWrist(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from parallel to vertical takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(700 * adjustment);
        wrist.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }
    public Progress moveWristForCapstone() {
        double target = WRIST_CAPSTONE;
        isWristParallel = true;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveWrist(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from parallel to vertical takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(700 * adjustment);
        wrist.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void grabberInit() {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_INIT);
    }

    public void grabberOpen () {
        if (grabber==null) return;
        if (armIsIn)
            grabber.setPosition(GRABBER_OPEN_IN);
        else
            grabber.setPosition(GRABBER_OPEN);
        isGrabberOpened = true;
    }
    public void grabberOpenAuto () {
        grabber.setPosition(GRABBER_OPEN);
        isGrabberOpened = true;
    }

    public void grabberReGrab () {
        if (grabber==null) return;
        grabberOpen();
        for(int i=0; i<1000; i++) // add some dummy delay
            grabber.getPosition();
        grabberClose();
    }

    public void grabberClose () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_CLOSE);
        isGrabberOpened = false;
    }

    public void grabberAuto() {
        if (isGrabberOpened)
            grabberClose();
        else
            grabberOpen();
    }

    public Progress moveGrabber(boolean closed) {
        double target = GRABBER_CLOSE;
        if (!closed) {
           if (armIsIn)
               target = GRABBER_OPEN_IN;
           else
               target = GRABBER_OPEN;
        }
        isGrabberOpened = !closed;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveGrabber(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from up to down takes 1 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(900 * adjustment);
        grabber.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void capstoneLeftInc() {
        double cur_pos = capstoneServo.getPosition();
        cur_pos -= WRIST_INC_UNIT;
        if (cur_pos<0) cur_pos=0;
        capstoneServo.setPosition(cur_pos);
        if (Math.abs(cur_pos-CAPSTONE_OUT)<0.2)
            isCapstoneServoOut = true;
        else
            isCapstoneServoOut = false;
    }

    public void capstoneRightInc() {
        double cur_pos = capstoneServo.getPosition();
        cur_pos += WRIST_INC_UNIT;
        if (cur_pos>1) cur_pos=1.0;
        capstoneServo.setPosition(cur_pos);
        if (Math.abs(cur_pos-CAPSTONE_OUT)<0.2)
            isCapstoneServoOut = true;
        else
            isCapstoneServoOut = false;
    }

    public void capstoneServoInit() {
        capstoneServo.setPosition(CAPSTONE_INIT);
        isCapstoneServoOut = false;
    }

    public void capstoneServoOut(){
        capstoneServo.setPosition(CAPSTONE_OUT);
        isCapstoneServoOut = true;
    }

    public void capstoneServoAuto(){
        if(isCapstoneServoOut)
            capstoneServoInit();
        else
            capstoneServoOut();
    }

    public void liftResetEncoder() {
        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

//    public void liftToPosition(int pos, double power) {
//        if (lifter==null) return;
//        if (Math.abs(lifter.getCurrentPosition()-pos)<20) return;
//
//        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lifter.setTargetPosition(pos);
//        lifter.setPower(power);
//    }

    public void liftUp (boolean slow, boolean force)  {
        if (lifter==null) return;
        // lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!force) {
            if (lifter.getCurrentPosition() > LIFT_MAX) {
                liftStop();
                return;
            }
        }
        if (slow || force)
            lifter.setPower(LIFT_POWER_SLOW);
        else
            lifter.setPower(LIFT_POWER);
    }

    public void liftDown(boolean slow, boolean force) {
        if (lifter==null) return;
        // lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!force) {
            if (lifter.getCurrentPosition() < LIFT_MIN) {
                liftStop();
                return;
            }
        }
        if (slow || force)
            lifter.setPower(-LIFT_POWER_SLOW_DOWN);
        else
            lifter.setPower(-LIFT_POWER);
    }

    public void liftStop() {
        if (lifter==null) return;
        // lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = lifter.getCurrentPosition();
        liftToPosition(pos, true);
        lifter.setPower(0);
    }

    public void liftToSafe() {
        liftToPosition(LIFT_SAFE_SWING, false);
    }

    public Progress liftToPosition(int pos, boolean slow) {
        if (lifter==null) return null;
        // if (Math.abs(lifter.getCurrentPosition()-pos)<20) return null;

        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(pos);
        lifter.setPower((slow?LIFT_POWER_SLOW:LIFT_POWER));
        return new Progress() {
            public boolean isDone() {
                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
            }
        };
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - arm.getPosition());
        arm.setPosition(position);
        if (position>=ARM_IN)
            armIsIn=true;
        else
            armIsIn=false;
        if (Math.abs(position-ARM_DOWN)<0.2)
            armIsDown = true;
        else
            armIsDown = false;
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 800);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void armOutComboAuto() {
        if (currentThread().isInterrupted()) return;
        armOutCombo();
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }
    }

    public void waitTM(double sec) {
        final String taskName = "Wait";
        if (!TaskManager.isComplete(taskName)) return;
        waitSec = sec;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                SGTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                };
            }
        }, taskName);
    }
    public void armOutCombo(){
        armOutCombo(0, true);
    }
    public void armOutCombo(double delaySec, boolean auto) {
        final String taskName = "Arm Out Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean grabIsOpened = isGrabberOpened;
        boolean armWasOut = !isArmInside();
        if (delaySec>0) {
            waitSec = delaySec;
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    };
                }
            }, taskName);
        }
        if (!armWasOut) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_SAFE);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    if (isWristParallel && isGrabberOpened) grabberClose();
                    if (arm.getPosition() > ARM_LOW) {
                        // arm inside the robot
                        int cur_pos = lifter.getCurrentPosition();
                        if (cur_pos < LIFT_SAFE_SWING) {
                            liftToPosition(LIFT_SAFE_SWING, false);
                        }
                    } else
                        liftToPosition(LIFT_GRAB, false);

                    return new Progress() {
                        @Override
                        public boolean isDone() {
                            return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                        }
                    };
                }
            }, taskName);
        }
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_AUTO);
                }
            }, taskName);
        } else if (armWasOut){
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_MORE);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                int cur_pos = lifter.getCurrentPosition();
                if (cur_pos<LIFT_SAFE_SWING) {
                    liftToPosition(LIFT_GRAB, false);
                }
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);
        if (grabIsOpened) { // restore grabber to open
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(false);
                }
            }, taskName);
        }
    }
    public void releaseStoneCombo() {
        final String taskName = "Release Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(false);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {

                int cur_pos = lifter.getCurrentPosition();
                int target_pos = cur_pos + 850;
                if(target_pos > LIFT_MAX)
                    target_pos = LIFT_MAX;
                liftToPosition(target_pos, true);

                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);
    }
    public void armInComboAuto(final boolean wristParallel) {
        if (currentThread().isInterrupted()) return;
        armInCombo(wristParallel, true);
        while (!TaskManager.isComplete("Arm In Combo")) {
            TaskManager.processTasks();
        }
    }
    public void armInCombo(final boolean wristParallel, boolean isAuto) {
        final String taskName = "Arm In Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true);
            }
        }, taskName);
        if (wristParallel!=isWristParallel) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    moveWrist(wristParallel);
                    grabberClose();
                    return new Progress() {
                        @Override
                        public boolean isDone() {
                            return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                        }
                    };
                }
            }, taskName);
        }
        if (lifter.getCurrentPosition()<LIFT_SAFE_SWING) {
            if (isAuto) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        liftToPosition(LIFT_SAFE_SWING_AUTO, false);
                        return new Progress() {
                            @Override
                            public boolean isDone() {
                                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                            }
                        };
                    }
                }, taskName);
            } else {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        liftToPosition(LIFT_SAFE_SWING, false);
                        return new Progress() {
                            @Override
                            public boolean isDone() {
                                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                            }
                        };
                    }
                }, taskName);
            }
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                liftToPosition(LIFT_DOWN, false);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                    }
                };

            }
        }, taskName);

        waitSec = 0.3;
        TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberOpen();
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    }; }
                    }, taskName);

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                return new Progress() {
                    @Override
                    public boolean isDone() { return true; }
                };
            }
        }, taskName);
    }

    public void grabStoneComboAuto() {
        if (currentThread().isInterrupted()) return;
        grabStoneCombo();
        while (!TaskManager.isComplete("Grab Stone Combo")) {
            TaskManager.processTasks();
        }
    }
    public void grabStoneCombo() { // grab stone from outside used by auto
        final String taskName = "Grab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN_GRAB, false);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                    }
                };
            }
        }, taskName);
        if (arm.getPosition() < ARM_OUT_AUTO) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT_AUTO);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress grabberProgress = moveGrabber(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return grabberProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                arm.setPosition(ARM_DELIVER);
                liftToPosition(LIFT_DOWN, false);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);

    }

    public void grabCapStoneCombo() {
        final String taskName = "Grab Cap Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean isLiftUp = (lifter.getCurrentPosition()>=LIFT_GRAB);
        if (!isLiftUp) { // stage-1
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(false); // open grabber
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_GRAB_READY_CAPSTONE,false);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_CAPSTONE);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(true); // close grabber
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    capstoneServoOut();
                    moveWristForCapstone();
                    return moveArm(ARM_CAPSTONE);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_UP_BEFORE_CAP, true);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_CAPSTONE_MORE);
                }
            }, taskName);
        }
        // else { // stage-2
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_UP_FINAL_CAP, true);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    capstoneServoInit();
                    return moveWrist(true);
                }
            }, taskName);
        // }
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                liftToPosition(LIFT_GRAB, false);
//                return new Progress() {
//                    @Override
//                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
//                    }
//                };
//            }
//        }, taskName);
//
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveArm(ARM_DOWN_FOR_CAP);
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveGrabber(false); // open grabber
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveArm(ARM_DOWN_SAFE);
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                liftToPosition(LIFT_DOWN_GRAB+50, false);
//                return new Progress() {
//                    @Override
//                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 150;
//                    }
//                };
//            }
//        }, taskName);
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                return moveGrabber(true); // close grabber
//            }
//        }, taskName);
    }

    public void grabStoneInsideCombo() {
        final String taskName = "Grab Stone Inside Combo";
        if (!TaskManager.isComplete(taskName)) return;

        // armInReadyGrabCombo();
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN, false);
                armDown();
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true);
            }
        }, taskName);
    }

    public void regrabStoneCombo(boolean moreIn) {
        final String taskName = "Regrab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return true;
                    }
                };
            }
        }, taskName);
        if (!moreIn) { // move out a little bit
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    liftToPosition(LIFT_GRAB, false);
                    return new Progress() {
                        @Override
                        public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                        }
                    };
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_FOR_CAP);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN_MORE);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(false); // open grabber
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN_GRAB, false);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(true); // close grabber
            }
        }, taskName);
    }

    public void armInReadyGrabCombo() {
        final String taskName = "Arm In Ready Grab Combo";
        if (!TaskManager.isComplete(taskName)) return;
        final boolean armWasIn = armIsIn;
        if (!armWasIn) { // move arm inside
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(true);
                }
            }, taskName);
            if (Math.abs(lifter.getCurrentPosition()-LIFT_SAFE_SWING)>400) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        int position = LIFT_SAFE_SWING+400;
                        liftToPosition(position, false);
                        return new Progress() {
                            @Override
                            public boolean isDone() {
                                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                            }
                        };
                    }
                }, taskName);
            }
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DOWN);
                }
            }, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    int position = LIFT_DOWN_GRAB;
                    liftToPosition(position, false);
                    return new Progress() {
                        @Override
                        public boolean isDone() {
                            return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                        }
                    };
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress wristProgress = moveWrist(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return wristProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                return moveArm(ARM_READY_GRAB);
            }
        }, taskName);
    }

    public void armInLiftUpReadyGrabCombo() {
        final String taskName = "Arm In LiftUp Ready Grab Combo";
        if (!TaskManager.isComplete(taskName)) return;
        final boolean armWasIn = armIsIn;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                int position = LIFT_SAFE_SWING_IN;
                if (armWasIn)
                    position = LIFT_SAFE_BRIDGE;
                liftToPosition(position, false);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress wristProgress = moveWrist(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return wristProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                return moveArm(ARM_DOWN);
            }
        }, taskName);
        if (!armWasIn) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    int position = LIFT_SAFE_BRIDGE;
                    liftToPosition(position, false);
                    return new Progress() {
                        @Override
                        public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                        }
                    };
                }
            }, taskName);
        }
    }

    public void deliverStoneComboAuto() {
        if (currentThread().isInterrupted()) return;
        deliverStoneCombo(true);
        while (!TaskManager.isComplete("Deliver Stone Combo")) {
            TaskManager.processTasks();
        }
    }

    public void deliverStoneCombo(boolean auto) {
        final String taskName = "Deliver Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DELIVER);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT);
                }
            }, taskName);
        }
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberOpen();
                    return liftToPosition(LIFT_SAFE_DELIVERY, false);
                }
            }, taskName);
        }
    }

    public void deliverStoneThrowComboAuto() {
        if (currentThread().isInterrupted()) return;
        deliverStoneThrowCombo();
        while (!TaskManager.isComplete("Deliver Stone Throw Combo")) {
            TaskManager.processTasks();
        }
    }

    public void deliverStoneThrowCombo() {
        final String taskName = "Deliver Stone Throw Combo";
        if (!TaskManager.isComplete(taskName)) return;

            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DELIVER);
                }
            }, taskName);

            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return liftToPosition(LIFT_SAFE_SWING-400, false);
                }
            }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                return moveArm(ARM_DELIVER_THROW);
            }
        }, taskName);
    }

    public void lifterDownCombo() {
        final String taskName = "Lifter Down Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftToPosition(LIFT_DOWN, false);
            }
        }, taskName);
    }
    /*
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (arm != null) {
            line.addData("arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
            line.addData("/armIn", "=%s", new Func<String>() {
                @Override
                public String value() {
                    return ((armIsIn?"T":"F"));
                }
            });
        }
        if (wrist != null) {
            line.addData("wrist", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return wrist.getPosition();
                }
            });
        }
        if (grabber != null) {
            line.addData("grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }
        if (lifter != null) {
            line.addData("lifter", "pos=%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return lifter.getCurrentPosition();
                }
            });
        }

    }
    public void grabStoneComboHigher(){
        final String taskName = "Grab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN_GRAB,true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_OUT_AUTO);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress grabberProgress = moveGrabber(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return grabberProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                arm.setPosition(ARM_DELIVER_HIGHER);
                liftToPosition(LIFT_DOWN+800,false);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);
    }

    public void grabStoneComboAutoHigher() {
        if (currentThread().isInterrupted()) return;
        grabStoneComboHigher();
        while (!TaskManager.isComplete("Grab Stone Combo")) {
            TaskManager.processTasks();
        }
    }
    public void deliverAndArmIn(){
        deliverStoneCombo(true);
        armInCombo(true, true);
    }

}



