/// License: See LICENSE file

extern crate mavlink;
use std::sync::Arc;
use std::thread;
use std::time::{SystemTime};

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


/*
px4_sitl supports a limited subset of mavlink messages:
MAVLINK_MSG_ID_HIL_SENSOR
MAVLINK_MSG_ID_HIL_GPS

MAVLINK_MSG_ID_RC_CHANNELS
MAVLINK_MSG_ID_DISTANCE_SENSOR
MAVLINK_MSG_ID_LANDING_TARGET
MAVLINK_MSG_ID_HIL_STATE_QUATERNION

MAVLINK_MSG_ID_HIL_OPTICAL_FLOW
MAVLINK_MSG_ID_ODOMETRY
MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:

*/
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

/// Create a heartbeat message
pub fn heartbeat_msg() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_PX4,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}


pub fn rc_channels_msg(sys_micros: u64,
) -> mavlink::common::MavMessage {

    let time_ms = (sys_micros / 1000) as u32;
    mavlink::common::MavMessage::RC_CHANNELS(mavlink::common::RC_CHANNELS_DATA {
        time_boot_ms: time_ms,
        chan1_raw: 128,
        chan2_raw: 128,
        chan3_raw: 128,
        chan4_raw: 128,
        chan5_raw: 128,
        chan6_raw: 128,
        chan7_raw: 128,
        chan8_raw: 128,
        chan9_raw: 128,
        chan10_raw: 128,
        chan11_raw: 128,
        chan12_raw: 128,
        chan13_raw: 128,
        chan14_raw: 128,
        chan15_raw: 128,
        chan16_raw: 128,
        chan17_raw: 128,
        chan18_raw: 128,
        chancount: 8,
        rssi: 100, //Receive signal strength indicator. Values: [0-100], 255: invalid/unknown.
        })
}

/// Create a HIL_STATE_QUATERNION message
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


pub fn hil_sensor_msg(sys_micros: u64, state: &mut VehicleState
) -> mavlink::common::MavMessage {
  // We need to align our fake abs_pressure with the fake GPS altitude,
  // or else the sensor fusion algos in autopilot will never align.

    let alt = state.alt.read();
    let wander = state.wander.read();

    mavlink::common::MavMessage::HIL_SENSOR(mavlink::common::HIL_SENSOR_DATA {
     time_usec: sys_micros , 
     xacc: state.xacc.read(),
     yacc: state.yacc.read(),
     zacc: state.zacc.read(),
     xgyro: state.xgyro.read(),
     ygyro: state.ygyro.read(),
     zgyro: state.zgyro.read(),
     xmag: state.xmag.read(),
     ymag: state.ymag.read(),
     zmag: state.zmag.read(),
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
    
    
/*

w	x	y	z	Description
1	0	0	0	Identity quaternion, no rotation
0	1	0	0	180° turn around X axis
0	0	1	0	180° turn around Y axis
0	0	0	1	180° turn around Z axis
sqrt(0.5)	sqrt(0.5)	0	0	90° rotation around X axis
sqrt(0.5)	0	sqrt(0.5)	0	90° rotation around Y axis
sqrt(0.5)	0	0	sqrt(0.5)	90° rotation around Z axis
sqrt(0.5)	-sqrt(0.5)	0	0	-90° rotation around X axis
sqrt(0.5)	0	-sqrt(0.5)	0	-90° rotation around Y axis
sqrt(0.5)	0	0	-sqrt(0.5)	-90° rotation around Z axis

*/
pub fn generate_attitude_quat(_state: &VehicleState) -> [f32; 4] {
  [0.0, 1.0, 0.0, 0.0]
}

fn calc_elapsed_micros(base_time: &SystemTime) -> u64 {  
  let elapsed = base_time.elapsed().unwrap();
  let elapsed_micros = (elapsed.as_secs() * 1000000) + (elapsed.subsec_micros() as u64);
  elapsed_micros
}

/// Some guesses as to accuracy of a fake accelerometer
const ACCEL_ABS_ERR : f32 = 1e-2;
const ACCEL_REL_ERR : f32 = 1e-4;

const GYRO_ABS_ERR : f32 = 1e-2;
const GYRO_REL_ERR : f32 = 1e-4;

const MAG_ABS_ERR : f32 = 1e-2;
const MAG_REL_ERR : f32 = 1e-4;

pub struct PhysicalVehicleState {
    local_x: f32,
    local_y: f32,
    local_z: f32,

    global_lat: f64,
    global_lon: f64,
    alt_msl: f32,

    vx: f32,
    vy: f32,
    vz: f32,
    quaternion: [f32; 4],
}

pub struct VehicleState {
    boot_time: SystemTime,

    lat: Sensulator,
    lon: Sensulator,
    alt: Sensulator,

    xgyro: Sensulator,
    ygyro: Sensulator,
    zgyro: Sensulator,

    xacc: Sensulator,
    yacc: Sensulator,
    zacc: Sensulator,

    xmag: Sensulator,
    ymag: Sensulator,
    zmag: Sensulator,

    wander: Sensulator,


}

//fn binary<T: Trait>(x: T, y: T) -> T

type VehicleConnectionRef = std::boxed::Box<dyn mavlink::MavConnection + std::marker::Send + std::marker::Sync>;


fn simulate_sensors_update(connection: &VehicleConnectionRef,
                           state: &mut VehicleState,
) {
    let mut sys_micros = calc_elapsed_micros(&state.boot_time);
    // altitude needs to change in lockstep in order for sensor fusion to align
    let common_alt: f32 = state.alt.read();

    for _medium_rate in 0..2 {
        for _fast_rate in 0..10 {
            // this message is required to be sent at high speed since it simulates the IMU
            connection.send_default( &hil_sensor_msg(sys_micros, state)).ok();

            sys_micros = calc_elapsed_micros(&state.boot_time);
        }

        connection.send_default( &hil_gps_msg(sys_micros,
                                        state.lat.read(),
                                        state.lon.read(),
                                        common_alt,
        )).ok();

//        connection.send_default(&rc_channels_msg(sys_micros,
//        )).ok();
    }

    //px4_sitl sends an early request for this msg
    connection.send_default(&hil_state_quaternion_msg(sys_micros,
                                                state.lat.read(),
                                                state.lon.read(),
                                                state.alt.read(),
                                                generate_attitude_quat(&state))).ok();


}
fn main() {

    let mut vehicle_state = VehicleState {
        boot_time: SystemTime::now(),

        lat: Sensulator::new(HOME_LAT, 1e-3, 1e-6),
        lon: Sensulator::new(HOME_LON, 1e-3, 1e-6),
        // this range appears to allow EKF fusion to begin
        alt: Sensulator::new(HOME_ALT, 10.0, 5.0),

        xgyro: Sensulator::new(0.001, GYRO_ABS_ERR, GYRO_REL_ERR),
        ygyro: Sensulator::new(0.001, GYRO_ABS_ERR, GYRO_REL_ERR),
        zgyro: Sensulator::new(0.001, GYRO_ABS_ERR, GYRO_REL_ERR),

        xacc:  Sensulator::new(0.001, ACCEL_ABS_ERR, ACCEL_REL_ERR),
        yacc:  Sensulator::new(0.001, ACCEL_ABS_ERR, ACCEL_REL_ERR),
        zacc:  Sensulator::new(9.8, ACCEL_ABS_ERR, ACCEL_REL_ERR),

        xmag: Sensulator::new(0.001, MAG_ABS_ERR, MAG_REL_ERR),
        ymag: Sensulator::new(0.001, MAG_ABS_ERR, MAG_REL_ERR),
        zmag: Sensulator::new(0.001, MAG_ABS_ERR, MAG_REL_ERR),

        wander: Sensulator::new(0.01, 1e-1, 1e-3),
    };



  let selector = "tcpout:rock64-03.local:4560";
  let connection = Arc::new(mavlink::connect(&selector).expect("Couldn't create new vehicle connection"));
  
  thread::spawn({
      let connection = connection.clone();

      move || {
          loop {
              //std::sync::Arc<std::boxed::Box<dyn mavlink::MavConnection + std::marker::Send + std::marker::Sync>>
              simulate_sensors_update(  &*connection,  &mut vehicle_state);

//              if let Ok(conn) = connection.downcast::<mavlink::MavConnection>() {
//                  simulate_sensors_update(
//                      &conn,
//                  &vehicle_state,
//                  );
//              }
              thread::yield_now();
          }
      }
  });

    // receiving loop continues in this thread
    loop {
      if let Ok((_header,msg)) = connection.recv() {
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




