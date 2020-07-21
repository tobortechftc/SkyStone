/**
 * FTC 2019-2020 SkyStone, TeleOp program, single driver + double drivers
 */

package org.firstinspires.ftc.teamcode.opmodes.ggMecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import com.qualcomm.robotcore.util.Range;


/**
 * Common codes 
 */

@TeleOp(name="Y20TeleEx", group="WT")
//@Disabled
public class Y20TeleEx extends Y20CommonEx {


   static boolean USE_CAR_MODE = false;     // disable carmode if false, spin the robot instead

   static boolean USE_LOW_SEN_DRIVE = true;
   boolean low_sen_drive_ = false;

   boolean double_driver_ = false;          // default false, change to true for double driver mode
   boolean driver1_override_enabled_ = true;       // driver1 override enabled if true, must be true for single driver mode

   static final double MIN_TELE_LOOP_TIME = 0.05;    // min loop time for TeleOp, any event less than could be missed
   static final boolean WORKAROUND_LIFT_MOTOR_DRIFT = true; 

   ///  Constructor
   public Y20TeleEx() {
   } 

   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      super.init(); 
      if( !double_driver_ ) driver1_override_enabled_ = true;
   }

   /// This method will be called repeated when the INIT button is pressed
   @Override public void init_loop() {
      super.init_loop(); 

      telemetry.addData("Y20TeleEx::init_loop()", "%s; Time: %.1fsec; loop_cnt=%d, loop_time= %.3fsec", (isRedTeam()?"RED":"BLUE"), timer_.seconds(),loop_cnt_,timer_.seconds()/loop_cnt_);
      if( range_stone_!=null ) telemetry.addData("StoneRange", "dist="+String.format("%.2fm", getRangeDist(RangeName.RANGE_STONE))+"; init="+String.format("%.2fm",range_stone_dist_init_));
      if( USE_RGBV3_FOR_STONE && rgb_range_stone_!=null ) telemetry.addData("StoneRgbV3Range", "dist="+String.format("%.2fm", getRangeDist(RangeName.RGB_RANGE_STONE))+"; init="+String.format("%.2fm",rgb_range_stone_dist_init_));
      telemetry.update();
   }

   /// This method will be called once when the PLAY button is first pressed.
   @Override public void start() {
      super.start(); 
   }


   /// This method will be called repeatedly in a loop
   @Override public void loop () {
      curr_time_ = timer_.time();
      loop_cnt_ ++; 

      if ((curr_time_ - last_button_time_) > MIN_BUTTON_INTERVAL) {
         if (gamepad1.x) {
            x1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.y) {
            y1_cnt_++;
            last_button_time_ = curr_time_;
         } else if ( (driver1_override_enabled_&&gamepad1.a) || (double_driver_&&gamepad2.a) ) {
            a1_cnt_++;
            last_button_a1_time_ = curr_time_;
            last_button_time_ = curr_time_;
         } else if ( (driver1_override_enabled_&&gamepad1.b) || (double_driver_&&gamepad2.b) ) {
            b1_cnt_++;
            last_button_time_ = curr_time_;
            last_button_b1_time_ = curr_time_;
         } else if (gamepad1.left_bumper) {
            lb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if ( (driver1_override_enabled_&&gamepad1.right_bumper) || (double_driver_&&gamepad2.right_bumper) ) {
            rb1_cnt_++;
            last_button_time_ = curr_time_;
            last_button_rb1_time_ = curr_time_;
         } else if (gamepad1.left_stick_button) {
            lsb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.right_stick_button) {
            rsb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if( gamepad1.left_trigger>0.5 && gamepad1.dpad_left ) {
            lt1_left_cnt_++;
            last_button_time_ = curr_time_;
         } else if( gamepad1.left_trigger>0.5 && gamepad1.dpad_right ) {
            lt1_right_cnt_++;
            last_button_time_ = curr_time_;
         } 
      } 

      double power_lf = 0, power_lb = 0, power_rf = 0, power_rb = 0;
      double power_sweeper = 0;
      double lsy = 0, lsx = 0, rsy = 0, rsx = 0;
      double drive_power_f = 1.0;

      rsx = gamepad1.right_stick_x ;

      /// Use Mecanum wheels by right joystick
      boolean USE_MECANUM_WHEELS = true;
      boolean USE_MECANUM_FOR_SIDEWALK_ONLY = false;

      low_sen_drive_ = ((lsb1_cnt_%2)==1) ? true : false; 

      if( !USE_MECANUM_WHEELS && Math.abs(rsx) > 0.1 ) {
         /// Rotate/spin the robot

         // Use right_stick_x to rotate the robot; left => turn left(CCW), right => turn right(CW)
         // Ignore it if the value is too small to avoid mis-operation.
         // right_stick_y is not used at this time
         power_rf = rsx;
         power_lf = rsx;

         // clip the power_rf/power_lf values so that the values never exceed +/- 1.0
         power_rf = Range.clip(power_rf, -1, 1);
         power_lf = Range.clip(power_lf, -1, 1);

         // scale the joystick value to make it easier to control the robot more precisely at slower speeds.
         power_rf = (double) scaleRotatePower(power_rf);
         power_lf = (double) scaleRotatePower(power_lf);

         power_rb = power_rf;
         power_lb = power_lf;


      } else {
         ///  Use left stick to move/turn the robot to a specific direction at a given speed

         lsy = -gamepad1.left_stick_y;   // throttle
         lsx = gamepad1.left_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster

         if( !USE_CAR_MODE && Math.abs(lsx)>=JOYSTICK_DEAD_ZONE && Math.abs(lsy)>=JOYSTICK_DEAD_ZONE ) {
            // force it into spin mode
            lsy = 0; 
         }
         power_lf = lsx * Math.abs(lsx) + lsy * Math.abs(lsy);
         power_rf = -lsx * Math.abs(lsx) + lsy * Math.abs(lsy);

         // clip the power_rf/power_lf values so that the values never exceed +/- 1.0
         power_rf = Range.clip(power_rf, -1, 1);
         power_lf = Range.clip(power_lf, -1, 1);

         // scale the joystick value to make it easier to control the robot more precisely at slower speeds.  
         if (USE_LOW_SEN_DRIVE && low_sen_drive_) {
            power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,/*drive_power_f*/1.0);
            power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,/*drive_power_f*/1.0);
         } else {
            power_rf = (double) scaleDrivePower(power_rf, drive_power_f);
            power_lf = (double) scaleDrivePower(power_lf, drive_power_f); 
         }

         power_rf = Range.clip(power_rf, -ENCODER_MAX_DRIVE_POWER, ENCODER_MAX_DRIVE_POWER);
         power_lf = Range.clip(power_lf, -ENCODER_MAX_DRIVE_POWER, ENCODER_MAX_DRIVE_POWER);

         /// LB is same as LF, RB is same as RF
         power_lb = power_lf;
         power_rb = power_rf;

//         boolean workaround_v53_carmode = true;
//         if(  Math.abs(lsx)<JOYSTICK_DEAD_ZONE ) {
//            // go forward/backward
//         } else if( WORKAROUND_V53_BUG && Math.abs(lsy)<JOYSTICK_DEAD_ZONE ) {
//            // spin the robot
//            // Workaround for weired rotating bug for Mechanum wheel, Thx, 2019/11/03
//            power_lb *= -1;
//            power_rf *= -1;
//         }
//         else if( USE_CAR_MODE && workaround_v53_carmode && Math.abs(lsx)>JOYSTICK_DEAD_ZONE && Math.abs(lsy)>JOYSTICK_DEAD_ZONE ) {
//            // car mode
//            boolean left_joystick_3modes = true;
//            double ls_ratio = lsx/lsy;
//            if( left_joystick_3modes || (Math.abs(ls_ratio)>0.5 && Math.abs(ls_ratio)<2) ) {
//               if( lsx>0 ) {
//                  if( lsy>0 ) {
//                     double t = power_rf;
//                     power_rf = -power_lb;
//                     power_lb = t;
//                  } else {
//                     double t = power_lf;
//                     power_lf = -power_rb;
//                     power_rb = -t;
//                  }
//                  power_lb*=-1;  // carmode, 2020/02/23
//               } else {
//                  if( lsy>0 ) {
//                     double t = power_rf;
//                     power_rf = power_lb;
//                     power_lb = -t;
//                  } else {
//                     double t = power_lf;
//                     power_lf = power_rb;
//                     power_rb = -t;
//                  }
//                  power_rb*=-1;   // carmode, 2020/02/23
//               }
//            }
//         }
      }

      boolean drive_sidewalk = true;
      if( USE_MECANUM_WHEELS && (Math.abs(lsx)+ Math.abs(lsy))<JOYSTICK_DEAD_ZONE ) {
         lsx = gamepad1.right_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster
         lsy = -gamepad1.right_stick_y;   // throttle
         if (USE_MECANUM_FOR_SIDEWALK_ONLY) lsy = 0;
         drive_sidewalk = true;

         power_lf = lsy+lsx;
         power_lb = lsy-lsx;
         power_rf = lsy-lsx;
         power_rb = lsy+lsx;

         power_lf = Range.clip(power_lf, -1, 1);
         power_lb = Range.clip(power_lb, -1, 1);
         power_rf = Range.clip(power_rf, -1, 1);
         power_rb = Range.clip(power_rb, -1, 1);

         if (USE_LOW_SEN_DRIVE && low_sen_drive_) {
            if(lsy<0.1) { // sidewalk
               power_rf = (double) scaleDrivePowerLowSensitivitySidewalk(power_rf,/*drive_power_f*/1.0);
               power_lf = (double) scaleDrivePowerLowSensitivitySidewalk(power_lf,/*drive_power_f*/1.0);
               power_rb = (double) scaleDrivePowerLowSensitivitySidewalk(power_rb,/*drive_power_f*/1.0);
               power_lb = (double) scaleDrivePowerLowSensitivitySidewalk(power_lb,/*drive_power_f*/1.0);
            } else { // forward/backward/diagonal
               power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,/*drive_power_f*/1.0);
               power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,/*drive_power_f*/1.0);
               power_rb = (double) scaleDrivePowerLowSensitivity(power_rb,/*drive_power_f*/1.0);
               power_lb = (double) scaleDrivePowerLowSensitivity(power_lb,/*drive_power_f*/1.0);
            }
         } else {
            power_lf = (double) scaleDrivePower(power_lf, drive_power_f);
            power_lb = (double) scaleDrivePower(power_lb, drive_power_f);
            power_rf = (double) scaleDrivePower(power_rf, drive_power_f);
            power_rb = (double) scaleDrivePower(power_rb, drive_power_f);
         }

         double max_mw_power = ENCODER_MAX_SIDEWALK_POWER;      // cap thse max wheel power for sidewalk
         power_lf = Range.clip(power_lf, -max_mw_power, max_mw_power);
         power_lb = Range.clip(power_lb, -max_mw_power, max_mw_power);
         power_rf = Range.clip(power_rf, -max_mw_power, max_mw_power);
         power_rb = Range.clip(power_rb, -max_mw_power, max_mw_power);
      }

      /// Flip robot if needed
      boolean flip_robot = false;  // grabber facing forward
      if (flip_robot) {
         double p = power_lf;
         power_lf = power_rf;
         power_rf = p;
         power_lb = power_lf;
         power_rb = power_rf;
      }

      /// Set power values for all motors after clipping
      power_lf = Range.clip(power_lf, -1, 1);
      power_lb = Range.clip(power_lb, -1, 1);
      power_rf = Range.clip(power_rf, -1, 1);
      power_rb = Range.clip(power_rb, -1, 1);

      motorRF_.setPower(power_rf);
      motorRB_.setPower(power_rb);
      motorLF_.setPower(power_lf);
      motorLB_.setPower(power_lb);



      if( USE_INTAKE ) {
         double intake_pwr = 0.0; 
         if( AUTO_START_INTAKE && b1_cnt_==0 && lb1_cnt_==0 && AUTO_START_INTAKE_TIME>AUTO_PUSH_TIME 
               && curr_time_-last_button_b1_time_>AUTO_START_INTAKE_TIME ) {
            // auto start intake after the stone is unloaded
            lb1_cnt_ = 1; 
         }
         if( lb1_cnt_%4 == 1 ) {
            intake_pwr = INTAKE_POWER; 
         } else if( lb1_cnt_%4 == 3 ) {
            intake_pwr = OUTTAKE_POWER; 
         }

         boolean stone_detected = false;
         boolean stone_detected_by_rgb = false;
         boolean stone_detected_by_range = false;
         boolean stone_detected_by_limit_switch = false;
         if( USE_RGB_FOR_STONE || USE_RANGE_FOR_STONE || USE_STONE_LIMIT_SWITCH || USE_RGBV3_FOR_STONE ) {
            if( USE_STONE_LIMIT_SWITCH && stone_limit_switch_!=null ) {
               if( !stone_limit_switch_.getState() ) {
                  stone_detected = true;
                  stone_detected_by_limit_switch = true; 
               }
            } 
            if( !stone_detected && USE_RGBV3_FOR_STONE && rgb_range_stone_!=null ) {
               double dis = getRangeDist(RangeName.RGB_RANGE_STONE);   // dflt 10Hz
               if( dis>MIN_RGB_RANGE_DIST ) {
                  if( dis<MAX_RGBV3_STONE_DIST ) {
                     stone_detected = true; 
                     stone_detected_by_range = true;
                  }
               }
            }
            if( !stone_detected && USE_RANGE_FOR_STONE && range_stone_!=null ) {
            //if( USE_RANGE_FOR_STONE && range_stone_!=null && (intake_pwr==INTAKE_POWER) ) {  // read only if intaking
               double dis = getRangeDist(RangeName.RANGE_STONE);   // dflt 10Hz
               if( dis>MIN_RANGE_DIST && dis<1.0 ) {
                  if( dis<MAX_RANGE_DIST_STONE ) {
                     stone_detected = true; 
                     stone_detected_by_range = true;
                  } else if( range_stone_dist_init_>MIN_RANGE_DIST && AUTO_CALIBRATE_RANGE ) {
                     if( range_stone_dist_init_>MIN_RANGE_DIST && dis<range_stone_dist_init_-DELTA_RANGE_DIST_STONE ) {
                        stone_detected = true;
                        stone_detected_by_range = true;
                     }
                  }
               }
            }

            if( USE_RGB_FOR_STONE && rev_rgb_range_!=null) {
               int alpha = rev_rgb_range_.alpha(); 
               double dist = rev_rgb_range_.getDistance(DistanceUnit.CM);
               if( !stone_detected && !Double.isNaN(dist) && dist>MIN_RANGE_DIST ) {
                  // check distance first, which is more reliable
                  if( AUTO_CALIBRATE_RGB_RANGE && rev_rgb_dist_init_>MIN_RANGE_DIST ) {
                     if( dist<rev_rgb_dist_init_-DELTA_RANGE_DIST_STONE ) {
                        stone_detected = true;
                        stone_detected_by_rgb = true;
                     }
                  } else {
                     if( dist<MAX_RGB_DIST_STONE ) {
                        stone_detected = true;
                        stone_detected_by_rgb = true;
                     }
                  }
               }
               if( !stone_detected && alpha>0 ) {
                  if( AUTO_CALIBRATE_RGB && rev_rgb_alpha_init_>=MIN_RGB_ALPHA ) {
                     if( alpha > rev_rgb_alpha_init_+DELTA_RGB_ALPHA_STONE ) {
                        stone_detected = true;
                        stone_detected_by_rgb = true;
                     }
                  } else {
                     if( alpha>MIN_RGB_ALPHA_STONE ) {
                        stone_detected = true;
                        stone_detected_by_rgb = true;
                     }
                  }
               }
            } 

            if( AUTO_SPIT_STONE2 ) {
               if( (stone_detected_by_rgb && stone_detected_by_range) ) {
                  // two stones detected
                  stone_detected = false;
                  lb1_cnt_ = 3; 
                  last_double_stone_time_ = curr_time_; 
               } else if( last_double_stone_time_>0.0 && curr_time_-last_double_stone_time_<AUTO_SPIT_STONE2_TIME ) {
                  stone_detected = false;
                  lb1_cnt_ = 3; 
               }
            }

            if( b1_cnt_%3==0 && stone_detected ) { // automatically hold the stone
               b1_cnt_ ++; 
               last_auto_hold_time_ = curr_time_; 
            }
            if( AUTO_GRAB_STONE 
                  && (stone_detected || (last_auto_hold_time_>0.0 && curr_time_-last_auto_hold_time_<1.0)) // fix stone hold bug, 03/10
                  && (curr_time_-last_auto_hold_time_>0.5)   // 0.5sec delay
                  && ( (rb1_cnt_%4)==0  || (rb1_cnt_%4)==3 )
                  && (b1_cnt_==1)
                  && Math.abs(servo_pusher_.getPosition()-PUSHER_HOLD)<0.02) {
               // Make sure pusher is in the HOLD position and stone is locked
               rb1_cnt_ = 1 ;  
               last_button_rb1_time_ = curr_time_;  
            }
         }


         if( lb1_cnt_==1 && b1_cnt_%3==2 ) {
            b1_cnt_ = 0;  // auto reset pusher and gater
         }

         servo_pusher_pos_ = PUSHER_INIT; 
         servo_gater_pos_ = GATER_CLOSE; 

         if( USE_ARM ) {
            // Basic control of arm/claw: X to change arm position, Y to change claw position
            int x1_mod = x1_cnt_%3 ; 
            if( x1_mod==0 ) {
               servo_arm_pos_ = ARM_COLLECT; 
            } else if( x1_mod==1 ) {
               servo_arm_pos_ = ARM_GRAB; 
            } else {
               servo_arm_pos_ = ARM_DELIVER; 
            }

            int rb1_mod = rb1_cnt_%4; 
            if( rb1_mod==0 ) {   
               // Collect stone: arm up, claw open
               if( x1_mod==0 ) {
                  servo_arm_pos_ = ARM_COLLECT; 
               }
               servo_claw_pos_ = CLAW_OPEN; 
            } else if( rb1_mod==1 ) { 
               // Grab stone: arm down, claw close after a small delay
               servo_arm_pos_ = ARM_GRAB; 
               if(curr_time_-last_button_rb1_time_ > 0.5) {
                  servo_claw_pos_ = CLAW_CLOSE; 
               } else {
                  servo_claw_pos_ = CLAW_OPEN; 
               }
               if( USE_CAPSTONE && gamepad1.left_trigger>0.5 /*&& gamepad1.y*/ ) { 
                  // manually release capstone
                  servo_claw_pos_ = CLAW_RELEASE_CAPSTONE; 
               }
            } else if( rb1_mod==2 ) {
               // Align stone: arm out, claw close
               servo_gater_pos_ = GATER_OPEN; 
               servo_pusher_pos_ = PUSHER_INIT; 
               if( curr_time_-last_button_rb1_time_ > 0.3 ) { 
                  // give gater a little time to open first, then move the stone out
                  // 0.2sec is tight, change to 0.3, 2020/03/07 
                  servo_arm_pos_ = ARM_DELIVER; 
                  if( b1_cnt_==1 ) { 
                     b1_cnt_ = 2 ; 
                     last_button_b1_time_ = 0.0; 
                  }
               } else { 
                  servo_arm_pos_ = ARM_GRAB; 
               }

               servo_claw_pos_ = CLAW_CLOSE; 
               if( USE_CAPSTONE && gamepad1.left_trigger>0.5 /*&& gamepad1.y*/ ) { // release capstone
                  servo_claw_pos_ = CLAW_RELEASE_CAPSTONE; 
               }

               // Adjust the lift height
               if( AUTO_LIFT_STONE && curr_time_-last_button_rb1_time_<MIN_TELE_LOOP_TIME ) {
               /*
                  if( last_stone_lift_enc_>0 ) {
                     a1_cnt_ = (last_stone_lift_enc_+LIFT_ENC_COUNT_PER_STONE/2)/LIFT_ENC_COUNT_PER_STONE+1; 
                  } else {
                     a1_cnt_ = 1; 
                  }
                  */
                  a1_cnt_ = last_stone_level_ + 1; 
                  last_button_a1_time_ = curr_time_;
               }
            } else if( rb1_mod==3 ) {
               // Deliver stone: arm out, claw open
               // Then reset arm/claw
               servo_gater_pos_ = GATER_OPEN; 
               servo_arm_pos_ = ARM_DELIVER; 
               servo_claw_pos_ = CLAW_OPEN; 

               if( curr_time_-last_button_rb1_time_<=0.2 ) {
                  // let stone settle
                  if( curr_time_-last_button_rb1_time_<=MIN_TELE_LOOP_TIME ) {
                     last_stone_lift_enc_ = getMotorEncoder(MotorName.MOTOR_LIFT);
                     //last_stone_level_ = last_stone_lift_enc_/LIFT_ENC_COUNT_PER_STONE + 1; 
                     last_stone_level_ = (last_stone_lift_enc_+LIFT_ENC_COUNT_PER_STONE/3)/LIFT_ENC_COUNT_PER_STONE + 1;  // fix the stone level boundary bug, 2020/03/11 
                  } 
               } else if( curr_time_-last_button_rb1_time_<=0.5 ) {
                  // rise lift
                  int t = (last_stone_lift_enc_+LIFT_ENC_COUNT_PER_STONE/2)/LIFT_ENC_COUNT_PER_STONE+1; 
                  if( t < MAX_LIFT_ENC_COUNT/LIFT_ENC_COUNT_PER_STONE+1  ) {
                     if( a1_cnt_ != t+1 ) {
                        a1_cnt_ = t+1; 
                        last_button_a1_time_ = curr_time_;
                     }
                  }
               } else if( curr_time_-last_button_rb1_time_<=0.8 ) {
                  servo_arm_pos_ = ARM_COLLECT; 
               } else {
                  // lower lift
                  int t = MAX_LIFT_ENC_COUNT/LIFT_ENC_COUNT_PER_STONE +2;
                  if( a1_cnt_ != t && a1_cnt_!=0 ) {
                     // bug: a1_cnt_ oscillate between 0 and M+2; fixed, 03/07
                     a1_cnt_ = t ;
                     last_button_a1_time_ = curr_time_;
                  } 
                  servo_arm_pos_ = ARM_COLLECT; 

                  servo_gater_pos_ = GATER_CLOSE; 
                  servo_pusher_pos_ = PUSHER_INIT; 
                  if( b1_cnt_==2 ) {
                     b1_cnt_ = 0 ;
                  }
               }
            }

            servo_arm_pos_ = Range.clip(servo_arm_pos_,0,1);
            servo_arm_.setPosition(servo_arm_pos_); 
            servo_claw_pos_ = Range.clip(servo_claw_pos_,0,1);
            servo_claw_.setPosition(servo_claw_pos_); 
         }

         if( USE_STONE_PUSHER ) {

            if( b1_cnt_%3==1 ) { // hold stone
               servo_pusher_pos_ = PUSHER_HOLD ; 
               //lb1_cnt_ = 0;  
               if( curr_time_-last_auto_hold_time_<MIN_TELE_LOOP_TIME ) {
                  // stop the intake, but allow driver to override it later.
                  lb1_cnt_ = 2;  
               }
            } else if( b1_cnt_%3==2 ) {  // unload/flush stone
               if( curr_time_-last_button_b1_time_ < 0.2 ) { // give gater a little time to open first, then push
                  servo_pusher_pos_ = PUSHER_HOLD ; 
                  if( curr_time_-last_button_b1_time_<0.1 && rb1_cnt_==1 ) { 
                     rb1_cnt_ = 0;       // rise arm and open claw to release the stone 
                  }
               } else { 
                  servo_pusher_pos_ = PUSHER_UNLOAD; 
               }
               servo_gater_pos_ = GATER_OPEN ; 
               lb1_cnt_ = 0; 
               if( AUTO_PUSH_TIME>0 && last_button_b1_time_>0 && curr_time_-last_button_b1_time_>AUTO_PUSH_TIME ) {
                  b1_cnt_ = 0; 
               }
            }
            servo_pusher_.setPosition(servo_pusher_pos_); 
            servo_gater_.setPosition(servo_gater_pos_); 
         }


         motor_left_intake_.setPower(intake_pwr); 
         motor_right_intake_.setPower(intake_pwr); 

         /// Lower/raise intake module
         double servo_pwr = CR_SERVO_STOP;
         if( intake_pwr==0.0 ) {  
            // don't lower/raise intake if wheels are turning to prevent damage
            if( gamepad1.left_trigger > 0.5 ) {
               servo_pwr = LOWER_INTAKE;
            } else if( gamepad1.right_trigger > 0.5 ) {
               servo_pwr = RAISE_INTAKE;
            }
         } 
         if( USE_INTAKE_MAG_SWITCH ) {
            boolean mag_s = intake_mag_switch_.getState() ;   // triggered / light ON => FALSE 
            if( !mag_s && servo_pwr==RAISE_INTAKE ) {  // triggered, intake already raised
               servo_pwr = CR_SERVO_STOP;             // cut the power
            } else if( servo_pwr==LOWER_INTAKE && mag_s ) {  // un-triggered, intake may already lowered
               if( intake_mag_change_time_==0 && intake_mag_prev_state_ ) {  // INTAKE already lowered when init
                  servo_pwr = CR_SERVO_STOP;          // cut the power 
               } else if( intake_mag_change_time_>0 && curr_time_-intake_mag_change_time_>LOWER_INTAKE_TIME ) {
                  servo_pwr = CR_SERVO_STOP;          // cut the power 
               }
            }
            if( mag_s != intake_mag_prev_state_ ) {
               intake_mag_prev_state_ = mag_s; 
               intake_mag_change_time_ = curr_time_; 
            }
         }
         servo_left_intake_.setPosition(servo_pwr);
         servo_right_intake_.setPosition(1-servo_pwr);
      } 

      if( USE_LIFT ) {
         int lift_enc = getMotorEncoder(MotorName.MOTOR_LIFT);
         int lift_tg_enc = 0; 
         boolean manual_lift_control = false;
         if( USE_RUN_TO_POS_FOR_LIFT ) {
            lift_tg_enc =  motor_lift_.getTargetPosition();
            //double lift_enc_delta = LIFT_ENC_COUNT_PER_STONE/2;  // 2inch incremental
            double lift_enc_delta = LIFT_ENC_COUNT_PER_STONE/4;   // 1inch incremental
            boolean allow_manual_control_when_motor_busy = true;
            if( allow_manual_control_when_motor_busy || !motor_lift_.isBusy() ) {
               if( (curr_time_-last_button_time_) > MIN_BUTTON_INTERVAL/3) {
                  if( (driver1_override_enabled_&&gamepad1.dpad_up) || (double_driver_&&gamepad2.dpad_up) ) { // 
                     if( lift_enc < MAX_LIFT_ENC_COUNT ) {
                        lift_tg_enc += lift_enc_delta;
                     } 
                     manual_lift_control = true;
                     a1_cnt_ = 0; 
                     last_button_time_ = curr_time_; 
                  } else if( (driver1_override_enabled_&&gamepad1.dpad_down) || (double_driver_&&gamepad2.dpad_down) ) {  // lower
                     if( lift_enc > 0 ) {
                        lift_tg_enc -= lift_enc_delta;
                     } 
                     manual_lift_control = true;
                     a1_cnt_ = 0; 
                     last_button_time_ = curr_time_; 
                  } 
               }
            }

            if( !manual_lift_control && a1_cnt_>0 ) { // use button to control lift
               if( a1_cnt_ > MAX_LIFT_ENC_COUNT/LIFT_ENC_COUNT_PER_STONE+1 ) {
                  a1_cnt_ = 0; 
                  last_button_a1_time_ = curr_time_; 
                  lift_tg_enc = MIN_LIFT_ENC_COUNT; 
               } else if( a1_cnt_ == MAX_LIFT_ENC_COUNT/LIFT_ENC_COUNT_PER_STONE+1 ) {
                  lift_tg_enc = MAX_LIFT_ENC_COUNT; 
               } else {
                  lift_tg_enc = (a1_cnt_-1)*LIFT_ENC_COUNT_PER_STONE + LIFT_ENC_COUNT_FIRST_STONE;
                  if( LIFT_RUN_TO_POS_RISE_TIME>0 && curr_time_-last_button_a1_time_<LIFT_RUN_TO_POS_RISE_TIME ) {
                     // Add 2inch to the encoder count to speedup the rise time, needed for 3in spool, 2020/01/26
                     //lift_tg_enc += LIFT_ENC_COUNT_PER_STONE/2;
                     // Update: not enough especially for high level, Thx, 2020/02/01 
                     // Increase encoder count by 20% till it reaches 0.1in below the targeted value, 2020/02/01
                     if( lift_enc<lift_tg_enc-LIFT_ENC_COUNT_PER_STONE/40 ) {
                        lift_tg_enc = lift_tg_enc*12/10; 
                     }
                  }
               }
            }
            if( !manual_lift_control && a1_cnt_==0 ) {
               // Fast fall
               if( last_lift_tg_enc_==MIN_LIFT_ENC_COUNT && 
                  ( (curr_time_-last_button_a1_time_)>LIFT_FAST_FALL_TIME ||   // upto 2sec
                  (lift_enc<0) ) )  // or 0 reached
               {
                  // give the lift a little time for lift to drop and then rewind the string to 0
                  lift_tg_enc = MIN_LIFT_ENC_COUNT/20;  
                  // overcome the motor drift issue
                  if( WORKAROUND_LIFT_MOTOR_DRIFT ) {
                     int n = last_stone_lift_enc_/LIFT_ENC_COUNT_PER_STONE+1; 
                     lift_tg_enc *= n; 
                  }
               } 
            }

            lift_tg_enc = (int)(Range.clip(lift_tg_enc,MIN_LIFT_ENC_COUNT,MAX_LIFT_ENC_COUNT));
            if( last_lift_tg_enc_ != lift_tg_enc ) {
               motor_lift_.setTargetPosition(lift_tg_enc);
               motor_lift_.setPower( LIFT_ENC_POWER );
               last_lift_tg_enc_ = lift_tg_enc;
            }
         } else { // RUN_USING_ENCOER for lift, which is faster
            power_lift_ = 0.0; 
            if( (driver1_override_enabled_&& (gamepad1.dpad_up || gamepad1.dpad_down)) ||
                (double_driver_ && (gamepad2.dpad_up || gamepad2.dpad_down) ) ) 
            {
               manual_lift_control = true;
               if( (driver1_override_enabled_&&gamepad1.dpad_up) || (double_driver_&&gamepad2.dpad_up) ) {
                  power_lift_ = LIFT_UP_POWER; 
               } else if( (driver1_override_enabled_&&gamepad1.dpad_down) || (double_driver_&&gamepad2.dpad_down) ) {
                  power_lift_ = LIFT_DOWN_POWER; 
               } 
            } 

            boolean auto_lift_ctrl = false;
            if( !manual_lift_control && a1_cnt_>0 ) { // use button to control lift
               auto_lift_ctrl = true;
               if( a1_cnt_ > MAX_LIFT_ENC_COUNT/LIFT_ENC_COUNT_PER_STONE+1 ) {
                  a1_cnt_ = 0; 
                  lift_tg_enc = MIN_LIFT_ENC_COUNT; 
               } else if( a1_cnt_ == MAX_LIFT_ENC_COUNT/LIFT_ENC_COUNT_PER_STONE+1 ) {
                  lift_tg_enc = MAX_LIFT_ENC_COUNT; 
               } else {
                  lift_tg_enc = (a1_cnt_-1)*LIFT_ENC_COUNT_PER_STONE + LIFT_ENC_COUNT_FIRST_STONE;
               }
            }

            if( auto_lift_ctrl ) {
               if( lift_enc < lift_tg_enc-LIFT_ENC_HOLD_COUNT ) {
                  power_lift_ = LIFT_UP_POWER; 
               } else if( lift_enc<lift_tg_enc+LIFT_ENC_HOLD_COUNT ) {
                  power_lift_ = LIFT_HOLD_POWER; 
               } else if( lift_enc<lift_tg_enc+2*LIFT_ENC_HOLD_COUNT ) {
                  power_lift_ = 0; 
               } else if( lift_enc<lift_tg_enc+3*LIFT_ENC_HOLD_COUNT ) {
                  power_lift_ = -LIFT_HOLD_POWER; 
               } else {  
                  power_lift_ = LIFT_DOWN_POWER; 
               }
            }

            if( lift_enc>=MAX_LIFT_ENC_COUNT && power_lift_*LIFT_UP_POWER>0 ) {
               power_lift_ = 0;   // over-extension protection
            } else if( lift_enc<=MIN_LIFT_ENC_COUNT && power_lift_*LIFT_DOWN_POWER>0 ) { 
               power_lift_ = 0;   // reverse protection
            } 

            power_lift_ = Range.clip(power_lift_,-1,1);
            motor_lift_.setPower( power_lift_ );
         }
      }

      if( USE_HOOKS ) {
         servo_left_hook_pos_ = LEFT_HOOK_UP; 
         servo_right_hook_pos_ = RIGHT_HOOK_UP; 
         //if( rsb1_cnt_%2 == 1 ) 
         if( y1_cnt_%2==1 ) {
            servo_left_hook_pos_ = LEFT_HOOK_DOWN; 
            servo_right_hook_pos_ = RIGHT_HOOK_DOWN; 
         }
         servo_left_hook_.setPosition(servo_left_hook_pos_); 
         servo_right_hook_.setPosition(servo_right_hook_pos_); 
      }

      if( USE_PARKING_STICKS ) {
         servo_left_park_pos_ = LEFT_PARK_IN; 
         servo_right_park_pos_ = RIGHT_PARK_IN; 
         if( lt1_left_cnt_%2 == 1 ) {
            servo_left_park_pos_ = LEFT_PARK_OUT; 
         }
         if( lt1_right_cnt_%2 == 1 ) {
            servo_right_park_pos_ = RIGHT_PARK_OUT; 
         }
         servo_left_park_.setPosition(servo_left_park_pos_); 
         servo_right_park_.setPosition(servo_right_park_pos_); 
      }


      boolean show_msg = true ;
      boolean show_title = true;
      boolean show_drive = true;
      boolean show_intake = USE_INTAKE && false; 
      boolean show_intake_mag_switch = USE_INTAKE_MAG_SWITCH && false; 
      boolean show_pusher_gater = false; 
      boolean show_stone_rgb = USE_RGB_FOR_STONE && false; 
      boolean show_stone_range = USE_RANGE_FOR_STONE && false; 
      boolean show_stone_rgb_range = USE_RGBV3_FOR_STONE && false; 
      boolean show_lift_pos = USE_LIFT && true; 
      boolean show_arm = USE_ARM && true; 
      boolean show_hooks = USE_HOOKS && false; 
      boolean show_parks = USE_PARKING_STICKS && false; 
      boolean show_limit_switch = USE_STONE_LIMIT_SWITCH && true;
      boolean show_odometry = true;

      int  TELEMTRY_MSG_FREQ = 10;      // show msg for every N loops to minimize impact on looptime
      if( show_msg && ((loop_cnt_%TELEMTRY_MSG_FREQ)==0) ) {
         if( show_title ) {
            telemetry.addData("Y20Tele: ", String.format("driver2=%s, drive1_override=%s; time=%.2fsec; loop_cnt=%d, loop_cycle=%.3fsec",String.valueOf(double_driver_),String.valueOf(driver1_override_enabled_),curr_time_,loop_cnt_,curr_time_/loop_cnt_)); 
         }
         if (show_odometry) {
            telemetry.addData("Vertical left encoder position:", verticalLeftEncoder.getCurrentPosition());
            telemetry.addData("Vertical right encoder position:", verticalRightEncoder.getCurrentPosition());
            telemetry.addData("horizontal encoder position:", horizontalEncoder.getCurrentPosition());
         }
         if( show_drive ) {
            telemetry.addData("MotorPower: ", String.format("lsx/lsy=%.2f/%.2f; power_lf/lb/rf/rb=%.2f/%.2f/%.2f/%.2f",lsx,lsy,power_lf,power_lb,power_rf,power_rb));
            telemetry.addData("MotorLFPower:", String.valueOf(motorLF_.getPower()));
            telemetry.addData("MotorLBPower:", String.valueOf(motorLB_.getPower()));
            telemetry.addData("MotorRFPower:", String.valueOf(motorRF_.getPower()));
            telemetry.addData("MotorRBPower:", String.valueOf(motorRB_.getPower()));
         } 
         if( show_intake ) {
            telemetry.addData("MotorLeftIntakePower:", String.valueOf(motor_left_intake_.getPower()));
            telemetry.addData("MotorRightIntakePower:", String.valueOf(motor_right_intake_.getPower()));
            telemetry.addData("ServoLeftIntakePower:", String.valueOf(servo_left_intake_.getPosition()));
            telemetry.addData("ServoRightIntakePower:", String.valueOf(servo_right_intake_.getPosition()));
            if( show_intake_mag_switch && intake_mag_switch_!=null ) {
               telemetry.addData("Intake Magnetic Switch", "state=" + String.valueOf(intake_mag_switch_.getState())+", last_change_time="+String.valueOf(intake_mag_change_time_)); 
            }
         } 
         if( show_pusher_gater ) {
            telemetry.addData("ServoPusher:", String.valueOf(servo_pusher_.getPosition()));
            telemetry.addData("ServoGater:", String.valueOf(servo_gater_.getPosition()));
         }
         if( show_stone_rgb ) {
            telemetry.addData("StoneColor", "red/green/blue/alpha||dist="+String.format("%d/%d/%d/%d||%.2fm", rev_rgb_range_.red(),rev_rgb_range_.green(),rev_rgb_range_.blue(),rev_rgb_range_.alpha(),rev_rgb_range_.getDistance(DistanceUnit.METER))+"; Init: alpha||dist="+String.format("%d||%.2fm",rev_rgb_alpha_init_,rev_rgb_dist_init_));
         }
         if( show_stone_rgb_range ) {
            telemetry.addData("StoneRgbV3Range", "dist="+String.format("%.2fm", getRangeDist(RangeName.RGB_RANGE_STONE))+"; init="+String.format("%.2fm",rgb_range_stone_dist_init_));
         }
         if( show_stone_range ) {
            telemetry.addData("StoneRange", "dist="+String.format("%.2fm", getRangeDist(RangeName.RANGE_STONE))+"; init="+String.format("%.2fm",range_stone_dist_init_));
         }
         if( USE_LIFT && show_lift_pos ) {
            if( USE_RUN_TO_POS_FOR_LIFT ) {
               telemetry.addData("Lift EncPos", ": curr/target="+String.valueOf(getMotorEncoder(MotorName.MOTOR_LIFT))+"/"+String.valueOf(motor_lift_.getTargetPosition())+", Power="+String.valueOf(motor_lift_.getPower())+", busy="+String.valueOf(motor_lift_.isBusy())+"; rb1_cnt_="+String.valueOf(rb1_cnt_)+", a1_cnt_="+String.valueOf(a1_cnt_)+", last_a1_time="+String.valueOf(last_button_a1_time_));
            } else {
               telemetry.addData("Lift control", "EncPos="+String.valueOf(getMotorEncoder(MotorName.MOTOR_LIFT))+", Power="+String.valueOf(power_lift_)+", busy="+String.valueOf(motor_lift_.isBusy()));
            }
         } 
         if( show_arm ) {
            telemetry.addData("ServoArm:", String.valueOf(servo_arm_.getPosition())+", arm_pos_="+String.valueOf(servo_arm_pos_));
            telemetry.addData("ServoClaw:", String.valueOf(servo_claw_.getPosition())+", claw_pos_="+String.valueOf(servo_claw_pos_));
            telemetry.addData("A1/B1/LB1/RB1:", String.valueOf(a1_cnt_)+"/"+String.valueOf(b1_cnt_)+"/"+String.valueOf(lb1_cnt_)+"/"+String.valueOf(rb1_cnt_)); 
            telemetry.addData("LastStone: last_stone_level_=", String.valueOf(last_stone_level_)+", last_stone_lift_enc_="+String.valueOf(last_stone_lift_enc_));

         }
         if( show_hooks ) {
            telemetry.addData("ServoHooks: Left/Right=", String.valueOf(servo_left_hook_.getPosition())+"/"+String.valueOf(servo_right_hook_.getPosition()));
         }
         if( show_parks ) {
            telemetry.addData("ServoParks: Left/Right=", String.valueOf(servo_left_park_.getPosition())+"/"+String.valueOf(servo_right_park_.getPosition())+"; lt1_left/right_cnt_: "+String.valueOf(lt1_left_cnt_)+"/"+String.valueOf(lt1_right_cnt_));
         }
         if( USE_INTAKE_MAG_SWITCH && intake_mag_switch_!=null ) telemetry.addData("IntakeMagSwitch", "state="+String.valueOf(intake_mag_switch_.getState())); 
         if( USE_STONE_LIMIT_SWITCH && stone_limit_switch_!=null ) telemetry.addData("StoneLimitSwitch", "state="+String.valueOf(stone_limit_switch_.getState())); 
      } 
   }







   /// Code to run when the op mode is first disabled goes here
   @Override public void stop () {

   }

   //ALL FUNCTIONS GO HERE!   


   /// Return current robot heading based on gyro/IMU reading



   /*
 * This method scales the joystick input so for low joystick values, the
 * scaled value is less than linear.  This is to make it easier to drive
 * the robot more precisely at slower speeds.
 */
   double scaleInput(double dVal) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

      // Get the corresponding index for the scaleInput array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      }
      if (index > 16) {
         index = 16;
      }


      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      return dScale;
   }


   /*
    * This method scales the joystick input so for low joystick values, the
    * scaled value is less than linear.  This is to make it easier to drive
    * the robot more precisely at slower speeds.
    */
   double scaleDrivePower(double dVal, double factor) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.34, 0.38, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00};  // Y17, with encoder

      // Get the corresponding index for the scaleDrivePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER;
      if( factor>0.0 && factor<=1.5) dScale *= factor;
      return dScale;
   }

   /// Overloaded vaersion with default factor of 1.0
   double scaleDrivePower(double dVal) {
      return scaleDrivePower(dVal, 1.0);
   }

   /// Low sensitivity drive mode for balancing and relic
   double scaleDrivePowerLowSensitivity(double dVal, double factor) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44};  // Y17, with encoder

      // Get the corresponding index for the scaleDrivePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER;
      if( factor>0.0 && factor<=1.5) dScale *= factor;
      return dScale;
   }

   /// Low sensitivity drive mode for balancing and relic
   double scaleDrivePowerLowSensitivitySidewalk(double dVal, double factor) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      //double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44};  // Y17, with encoder
      double[] scaleArray = {0.0, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52};  // Y20, TRMec

      // Get the corresponding index for the scaleDrivePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER;
      if( factor>0.0 && factor<=1.5) dScale *= factor;
      return dScale;
   }

   /// Scale the robot rotating power
   double scaleRotatePower(double dVal) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      double[] scaleArray = {0.0, 0.35, 0.4, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.78, 0.80, 0.85, 0.90, 0.93, 0.97, 1.0};   // Y17, with encoder for Mecanum 6

      // Get the corresponding index for the scalePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_ROTATE_POWER;
      return dScale;
   }

} 
