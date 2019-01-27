/// License: See LICENSE file

extern crate mavlink;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime};

use sensulator::Sensulator;


const STD_PRESS: f64 = 101325.0;  // static pressure at sea level (Pa)
const STD_TEMP: f64 = 288.15;    // standard temperature at sea level (K)
const LAPSE_RATE: f64 = -0.0065;   // standard temp altitude lapse rate (K/m)
const MOL_MASS : f64 = 0.0289644;  // molar mass of Earth's air (kg/mol)
const ACCEL_G : f64 = 9.80665;    // gravity acceleration (m/s^2)
const GAS_CONSTANT_R : f64 = 8.31432;    // universal gas constant, R


/// Convert altitude (meters) to standard barometric pressure (Pascals)
/// Note: this formula is likely only useful under 10k feet
fn altitude_to_baro_pressure(alt: f32) -> f32 {
  let big_alt: f64 = alt.into();
  let base = STD_TEMP / (STD_TEMP + (LAPSE_RATE * big_alt));
  let exp = (ACCEL_G * MOL_MASS) / (GAS_CONSTANT_R * LAPSE_RATE);
  let val: f64 = STD_PRESS * base.powf(exp);
  (val as f32)
}
    


/// Fake home coordinates
const HOME_LAT: f32 = 37.8;
const HOME_LON: f32 = -122.2;
const HOME_ALT: f32 = 500.0;


/// Create a message requesting the parameters list
pub fn request_parameters() -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::PARAM_REQUEST_LIST(mavlink::common::PARAM_REQUEST_LIST_DATA {
      target_system: 0,
      target_component: 0,
  })
}

/// Create a message enabling data streaming
pub fn request_stream() -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::REQUEST_DATA_STREAM(mavlink::common::REQUEST_DATA_STREAM_DATA {
      target_system: 0,
      target_component: 0,
      req_stream_id: 0,
      req_message_rate: 10,
      start_stop: 1,
  })
}

pub fn hil_state_quaternion_msg(sys_micros: u64,  
  lat: f32, 
  lon: f32,
  alt: f32,
  quat: [f32; 4],
) -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::HIL_STATE_QUATERNION(mavlink::common::HIL_STATE_QUATERNION_DATA {
    time_usec: sys_micros,
    attitude_quaternion: quat, 
    rollspeed: 0.0,
    pitchspeed: 0.0,
    yawspeed: 0.0,
    lat: (lat * 1E7) as i32,
    lon: (lon * 1E7) as i32,
    alt: (alt * 1E3) as i32,
    vx: 0,
    vy: 0,
    vz: 0,
     ind_airspeed: 0,
     true_airspeed: 0,
    xacc: 0,
    yacc: 0,
    zacc: 0,
  })

}


pub fn hil_sensor_msg(sys_micros: u64, 
  xacc: f32,
  yacc: f32,
  zacc: f32,
  alt: f32,
  wander: f32
) -> mavlink::common::MavMessage {
  // We need to align our fake abs_pressure with the fake GPS altitude,
  // or else the sensor fusion algos in autopilot will never align. 
  mavlink::common::MavMessage::HIL_SENSOR(mavlink::common::HIL_SENSOR_DATA {
     time_usec: sys_micros , 
     xacc: xacc,
     yacc: yacc,
     zacc: zacc,
     xgyro: wander,
     ygyro: wander,
     zgyro: wander,
     xmag: wander,
     ymag: wander,
     zmag: wander,
     abs_pressure: altitude_to_baro_pressure(alt),
     diff_pressure: 0.1 + wander,
     pressure_alt: alt,
     temperature: 25.0 + wander,
    // uint32_t fields_updated; /*<  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.*/
     fields_updated: 0,
  })
}

pub fn hil_gps_msg(sys_micros: u64, lat: f32, lon: f32, alt: f32
) -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::HIL_GPS(mavlink::common::HIL_GPS_DATA {
   time_usec: sys_micros,
   lat: (lat * 1E7) as i32,
   lon: (lon * 1E7) as i32,
   alt: (alt * 1E3) as i32,
   eph: 1,
   epv: 1,
   vel: 0,
   vn: 0,
   ve: 0,
   vd: 0,
   cog: 0,
   fix_type: 3,
   satellites_visible: 10,
  })
}
    
    
fn calc_elapsed_micros(base_time: &SystemTime) -> u64 {  
  let elapsed = base_time.elapsed().unwrap();
  let elapsed_micros = (elapsed.as_secs() * 1000000) + (elapsed.subsec_micros() as u64);
  elapsed_micros
}

/// Some guesses as to accuracy of a fake accelerometer
const ACCEL_ABS_ERR : f32 = 1e-2;
const ACCEL_REL_ERR : f32 = 1e-4;

fn main() {
  
  let boot_time = SystemTime::now();
  
  let mut fake_gps_lat = Sensulator::new(HOME_LAT, 1e-3, 1e-6);
  let mut fake_gps_lon = Sensulator::new(HOME_LON, 1e-3, 1e-6);
  // this range appears to allow EKF fusion to begin
  let mut fake_alt = Sensulator::new(HOME_ALT, 10.0, 5.0);

  let mut fake_xacc = Sensulator::new(0.001, ACCEL_ABS_ERR, ACCEL_REL_ERR);
  let mut fake_yacc = Sensulator::new(0.001, ACCEL_ABS_ERR, ACCEL_REL_ERR);

  //assume z is down
  let mut fake_zacc = Sensulator::new(9.8, ACCEL_ABS_ERR, ACCEL_REL_ERR);

  //a wandering random sensor
  let mut wander = Sensulator::new(0.01, 1e-1, 1e-3);


  let selector = "tcp:rock64-03.local:4560";
  let vehicle = Arc::new(mavlink::connect(&selector).unwrap());
  
  vehicle.send_default( &request_parameters()).unwrap();
  vehicle.send_default( &request_stream()).unwrap();

  let mut sys_micros: u64 = 0;
  
    thread::spawn({
        let vehicle = vehicle.clone();
        move || {
            loop {
              sys_micros = calc_elapsed_micros(&boot_time);
              let common_alt: f32 = fake_alt.read();
              
              for _x in 0..10 {
                // this message is required to be sent at high speed since it simulates the IMU
                vehicle.send_default(&hil_sensor_msg(sys_micros,
                  fake_xacc.read(),
                  fake_yacc.read(),
                  fake_zacc.read(),
                  common_alt,
                  wander.read()
                  )).ok();
                  
                  sys_micros = calc_elapsed_micros(&boot_time);
              }
                
              vehicle.send_default(&hil_gps_msg(sys_micros,
                fake_gps_lat.read(),
                fake_gps_lon.read(),
                common_alt,
                )).ok();
                
                thread::yield_now();
            }
        }
    });

    loop {
      if let Ok((_header,msg)) = vehicle.recv() {
        match msg.message_id() {
          //TODO use constants instead
          93 => continue, //HIL_ACTUATOR_CONTROLS
          _ => println!("{:?}", msg)
        }
      } 
      else {
        println!("bogus msg");
          continue;
      }
      thread::yield_now();
    }
}




