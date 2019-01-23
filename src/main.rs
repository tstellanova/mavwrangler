extern crate mavlink;
use std::sync::Arc;
use std::thread;
// use std::env;
use std::time::{SystemTime};

use sensulator::Sensulator;


const Pb: f64 = 101325.0;  // static pressure at sea level [Pa]
const Tb: f64 = 288.15;    // standard temperature at sea level [K]
const Lb: f64 = -0.0065;   // standard temperature lapse rate [K/m]
const M : f64 = 0.0289644;  // molar mass of Earth's air [kg/mol]
const G : f64 = 9.80665;    // gravity
const R : f64 = 8.31432;    // universal gas constant


fn altitude_to_baro_pressure(alt: f32) -> f32 {
  let big_alt: f64 = alt.into();
  let base = Tb / (Tb + (Lb * big_alt));
  let exp = (G * M) / (R * Lb);
  let val: f64 = Pb * base.powf(exp);
  (val as f32)
}
    

fn heartbeat_msg() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: 6,
        autopilot: 8,
        base_mode: 0,
        system_status: 0,
        mavlink_version: 3,
    })
}

const HOME_LAT: f32 = 37.8;
const HOME_LON: f32 = -122.2;
const HOME_ALT: f32 = 5000.0;





fn hil_state_quaternion_msg(sys_micros: u64,  
  lat: f32, 
  lon: f32,
  alt: f32,
  quat: Vec<f32>,
) -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::HIL_STATE_QUATERNION(mavlink::common::HIL_STATE_QUATERNION_DATA {
    time_usec: sys_micros,
    attitude_quaternion: quat, //vec![0.0, 0.0, 0.0, 0.0], //Vec<f32> /* 4 */,
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


fn hil_sensor_msg(sys_micros: u64, 
  xacc: f32,
  yacc: f32,
  zacc: f32,
  alt: f32,
  wander: f32
) -> mavlink::common::MavMessage {
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

fn hil_gps_msg(sys_micros: u64, lat: f32, lon: f32, alt: f32) -> mavlink::common::MavMessage {
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
    
fn main() {
  
  let boot_time = SystemTime::now();
  
  let mut fake_gps_lat = Sensulator::new();
  fake_gps_lat.set_absolute_error_range(1e-3);
  fake_gps_lat.set_relative_error(1e-6);
  fake_gps_lat.set_center_value(HOME_LAT);
  
  let mut fake_gps_lon = Sensulator::new();
  fake_gps_lon.set_absolute_error_range(1e-3);
  fake_gps_lon.set_relative_error(1e-6);
  fake_gps_lon.set_center_value(HOME_LON);
  
  // this range appears to allow EKF fusion to begin
  let mut fake_alt = Sensulator::new();
  fake_alt.set_absolute_error_range(10.0);
  fake_alt.set_relative_error(5.0);
  fake_alt.set_center_value(HOME_ALT);
  
  let mut fake_xacc = Sensulator::new();
  fake_xacc.set_absolute_error_range(1e-2);
  fake_xacc.set_relative_error(1e-4);
  fake_xacc.set_center_value(0.1);
  
  let mut fake_yacc = Sensulator::new();
  fake_yacc.set_absolute_error_range(1e-2);
  fake_yacc.set_relative_error(1e-4);
  fake_yacc.set_center_value(0.1);
  
  let mut fake_zacc = Sensulator::new();
  fake_zacc.set_absolute_error_range(1e-2);
  fake_zacc.set_relative_error(1e-4);
  fake_zacc.set_center_value(9.8);
  
  let mut wander = Sensulator::new();
  wander.set_absolute_error_range(1e-1);
  wander.set_relative_error(1e-3);
  wander.set_center_value(0.01);
  

  
    // let args: Vec<_> = env::args().collect();
    //
    // if args.len() < 2 {
    //     println!("Usage: mavlink-dump (tcp|udpin|udpout):ip:port");
    //     // return;
    // }
    // else {
    //   selector = &args[1];
    // }

    let selector = "tcp:rock64-03.local:4560";
    let vehicle = Arc::new(mavlink::connect(&selector).unwrap());
    
    vehicle.send(&mavlink::request_parameters()).unwrap();
    vehicle.send(&mavlink::request_stream()).unwrap();

    thread::spawn({
        let vehicle = vehicle.clone();
        move || {
            loop {
              //println!("elapsed: {:?}", boot_time.elapsed().unwrap());
              let elapsed = boot_time.elapsed().unwrap();
              let sys_micros: u64 = (elapsed.as_secs() * 1000000) + (elapsed.subsec_micros() as u64);
              let common_alt: f32 = fake_alt.read();
              
              // vehicle.send(&mavlink::heartbeat_message()).ok();
              // vehicle.send(&heartbeat_msg()).ok();
              vehicle.send(&hil_sensor_msg(sys_micros+2,
                fake_xacc.read(),
                fake_yacc.read(),
                fake_zacc.read(),
                common_alt,
                wander.read()
                )).ok();
                
              vehicle.send(&hil_gps_msg(sys_micros+4,
                fake_gps_lat.read(),
                fake_gps_lon.read(),
                common_alt,
                )).ok();
                
              // let quat = vec![wander.read(), wander.read(), wander.read(), wander.read()];
              // vehicle.send(&hil_state_quaternion_msg(sys_micros+6,
              //   fake_gps_lat.read(),
              //   fake_gps_lon.read(),
              //   legit_height,
              //   quat,
              //   )).ok();
            }
        }
    });

    loop {
        if let Ok(msg) = vehicle.recv() {
            println!("{:?}", msg);
        } else {
            break;
        }
    }
}
