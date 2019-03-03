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
pub fn hil_state_quaternion_msg(sys_micros: u64,  state: &mut VehicleState
) -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::HIL_STATE_QUATERNION(mavlink::common::HIL_STATE_QUATERNION_DATA {
    time_usec: sys_micros,
    attitude_quaternion: state.phyiscal_state.att_quat,
    rollspeed: state.phyiscal_state.rollspeed,
    pitchspeed: state.phyiscal_state.pitchspeed,
    yawspeed: state.phyiscal_state.yawspeed,
    lat: ( state.phyiscal_state.global_lat * 1E7) as i32,
    lon: (state.phyiscal_state.global_lon * 1E7) as i32,
    alt: (state.phyiscal_state.alt_wgs84 * 1E3) as i32,
    vx: (state.phyiscal_state.body_vx * 1E2) as i16,
    vy: (state.phyiscal_state.body_vy * 1E2) as i16,
    vz: (state.phyiscal_state.body_vz * 1E2) as i16,
    ind_airspeed: 0,
    true_airspeed: (state.airspeed.read() * 1E2) as u16,
      //TOOD convert these
    xacc: state.phyiscal_state.body_ax as i16,
    yacc: state.phyiscal_state.body_ay as i16,
    zacc: state.phyiscal_state.body_az as i16,
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

pub fn hil_gps_msg(sys_micros: u64, state: &mut VehicleState
) -> mavlink::common::MavMessage {
  mavlink::common::MavMessage::HIL_GPS(mavlink::common::HIL_GPS_DATA {
   time_usec: sys_micros,
   lat: (state.lat.read() * 1E7) as i32,
   lon: (state.lon.read() * 1E7) as i32,
   alt: (state.alt.read() * 1E3) as i32,
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
    

//TODO track attitude quaternion:
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


fn micros_from_duration(duration: &Duration) -> u64 {
    (duration.as_secs() * 1000000) + (duration.subsec_micros() as u64)
}

const SIM_TIME_MULTIPLIER: u32 = 4;

static mut SIM_CLOCK_TIME: Duration = Duration::from_secs(0);
static mut LAST_REAL_CLOCK_TIME: Duration = Duration::from_secs(0);
static mut SIM_ELAPSED_DELTA: Duration = Duration::from_secs(0);


fn calc_elapsed_micros(base_time: &SystemTime) -> u64 {
    let old_real_time = unsafe { LAST_REAL_CLOCK_TIME };
    let old_sim_clock_time = unsafe { SIM_CLOCK_TIME };
    let new_real_time = base_time.elapsed().unwrap();
    let real_delta = new_real_time - old_real_time;

    let sim_delta = SIM_TIME_MULTIPLIER * real_delta;
    let sim_new_time = old_sim_clock_time + sim_delta;
    let sim_elapsed_micros =  micros_from_duration(&sim_new_time);
    unsafe {
        LAST_REAL_CLOCK_TIME = new_real_time;
        SIM_ELAPSED_DELTA = sim_delta;
        SIM_CLOCK_TIME = sim_new_time;
    }
    sim_elapsed_micros
}



/// Some guesses as to accuracy of a fake accelerometer
const ACCEL_ABS_ERR : f32 = 1e-2;
const ACCEL_REL_ERR : f32 = 1e-4;

const GYRO_ABS_ERR : f32 = 1e-2;
const GYRO_REL_ERR : f32 = 1e-4;

const MAG_ABS_ERR : f32 = 1e-2;
const MAG_REL_ERR : f32 = 1e-4;


type Meters = f32;
type MetersPerSecond = f32;
type MetersPerSecondPerSecond = f32;
type RadiansPerSecond = f32;
type RadiansPerSecondPerSecond = f32;

type WGS84Degrees = f64;

/// Fake home coordinates
const HOME_LAT: WGS84Degrees = 37.8;
const HOME_LON: WGS84Degrees = -122.2;
const HOME_ALT: Meters = 500.0;



pub struct PhysicalVehicleState {
    /// --- Position -----
    local_x: Meters,
    local_y: Meters,
    local_z: Meters,

    global_lat: WGS84Degrees,
    global_lon: WGS84Degrees,
    alt_wgs84: Meters,

    /// Velocities in body frame
    body_vx: MetersPerSecond,
    body_vy: MetersPerSecond,
    body_vz: MetersPerSecond,

    /// Acceleration in body frame
    body_ax: MetersPerSecondPerSecond,
    body_ay: MetersPerSecondPerSecond,
    body_az: MetersPerSecondPerSecond,

    /// ---- Attitude --------
    /// W, X, Y, Z attitude quaternion
    att_quat: [f32; 4],
    /// Roll angular speed in rad/s
    rollspeed: RadiansPerSecond,
    /// Pitch angular speed in rad/s
    pitchspeed: RadiansPerSecond,
    /// Yaw angular speed in rad/s
    yawspeed: RadiansPerSecond,

    roll_accel: RadiansPerSecondPerSecond,
    pitch_accel: RadiansPerSecondPerSecond,
    yaw_accel: RadiansPerSecondPerSecond,

}

impl PhysicalVehicleState{
    fn new() -> PhysicalVehicleState {
        PhysicalVehicleState {
            local_x: 0.0,
            local_y: 0.0,
            local_z: 0.0,

            global_lat: HOME_LAT,
            global_lon: HOME_LON,
            alt_wgs84: HOME_ALT,

            /// Velocities in body frame
            body_vx: 0.0,
            body_vy: 0.0,
            body_vz: 0.0,

            /// Acceleration in body frame
            body_ax: 0.0,
            body_ay: 0.0,
            body_az: 0.0,

            /// ---- Attitude --------
            /// W, X, Y, Z attitude quaternion
            att_quat: [0.0, 0.0, 0.0, 0.0],
            /// Roll angular speed in rad/s
            rollspeed: 0.0,
            /// Pitch angular speed in rad/s
            pitchspeed: 0.0,
            /// Yaw angular speed in rad/s
            yawspeed: 0.0,

            roll_accel: 0.0,
            pitch_accel: 0.0,
            yaw_accel: 0.0,
        }
    }
}

pub struct VehicleState {
    boot_time: SystemTime,

    phyiscal_state: PhysicalVehicleState,

    ///--- Data arriving directly from sensors:
    /// GPS
    lat: Sensulator,
    lon: Sensulator,
    alt: Sensulator,

    /// Gyro
    xgyro: Sensulator,
    ygyro: Sensulator,
    zgyro: Sensulator,

    /// Accelerometer
    xacc: Sensulator,
    yacc: Sensulator,
    zacc: Sensulator,

    /// Magnetometer
    xmag: Sensulator,
    ymag: Sensulator,
    zmag: Sensulator,

    airspeed: Sensulator,

    wander: Sensulator,


}

//TODO use generid instead
type VehicleConnectionRef = std::boxed::Box<dyn mavlink::MavConnection + std::marker::Send + std::marker::Sync>;



fn send_slow_sensors(connection: &VehicleConnectionRef,
                     state: &mut VehicleState,
                     simulated_time: u64 ) {
    connection.send_default( &hil_gps_msg(simulated_time, state)).ok();
}

fn send_fast_sensors(connection: &VehicleConnectionRef,
                     state: &mut VehicleState,
                     simulated_time: u64 ) {

    // this message is required to be sent at high speed since it simulates the IMU
    connection.send_default( &hil_sensor_msg(simulated_time, state)).ok();
}


static mut LAST_SIM_SLOW_SENSORS_UPDATE: Duration = Duration::from_secs(0);

const MAX_IMU_DELAY_USEC:u64 =  125;

fn simulate_sensors_update(connection: &VehicleConnectionRef,
                           state: &mut VehicleState,
) {
    let last_sim_time = unsafe {SIM_CLOCK_TIME};
    let total_sim_delta = unsafe { SIM_ELAPSED_DELTA};
    let new_sim_time = calc_elapsed_micros(&state.boot_time);
    //println!("total_sim_delta: {:?}", total_sim_delta);

    //send multiple intermediate msgs to fill the gap
    let num_increments: u32 = 10;
    let incr_delta = total_sim_delta / num_increments;

    for i in 0..num_increments {
        let intermediate_sim_time = last_sim_time + (i*incr_delta);
        let intermediate_micros = micros_from_duration(&intermediate_sim_time);
        send_fast_sensors(connection, state, intermediate_micros);
    }
    send_fast_sensors(connection, state, new_sim_time);


    let slow_check_delta = unsafe { SIM_CLOCK_TIME - LAST_SIM_SLOW_SENSORS_UPDATE};
    if slow_check_delta.as_secs() > 0 {
        send_slow_sensors(connection, state, new_sim_time);
        unsafe {
            LAST_SIM_SLOW_SENSORS_UPDATE = SIM_CLOCK_TIME;
        }
    }

}

fn main() {

    let mut vehicle_state = VehicleState {
        boot_time: SystemTime::now(),
        phyiscal_state: PhysicalVehicleState::new(),

        lat: Sensulator::new(HOME_LAT as f32, 1e-3, 1e-6),
        lon: Sensulator::new(HOME_LON as f32, 1e-3, 1e-6),
        // this range appears to allow EKF fusion to begin
        alt: Sensulator::new(HOME_ALT as f32, 10.0, 5.0),

        xgyro: Sensulator::new(0.001, GYRO_ABS_ERR, GYRO_REL_ERR),
        ygyro: Sensulator::new(0.001, GYRO_ABS_ERR, GYRO_REL_ERR),
        zgyro: Sensulator::new(0.001, GYRO_ABS_ERR, GYRO_REL_ERR),

        xacc:  Sensulator::new(0.001, ACCEL_ABS_ERR, ACCEL_REL_ERR),
        yacc:  Sensulator::new(0.001, ACCEL_ABS_ERR, ACCEL_REL_ERR),
        zacc:  Sensulator::new(9.8, ACCEL_ABS_ERR, ACCEL_REL_ERR),

        xmag: Sensulator::new(0.001, MAG_ABS_ERR, MAG_REL_ERR),
        ymag: Sensulator::new(0.001, MAG_ABS_ERR, MAG_REL_ERR),
        zmag: Sensulator::new(0.001, MAG_ABS_ERR, MAG_REL_ERR),

        airspeed: Sensulator::new(0.001, 1e-3, 1e-6),

        wander: Sensulator::new(0.01, 1e-1, 1e-3),
    };


  //let selector = "tcpout:127.0.0.1:4560";
  //let selector = "serial:/dev/cu.usbserial-DA00C97E:57600";
  let selector = "tcpout:127.0.0.1:4560";

  let mut mavconn = mavlink::connect(&selector).expect("Couldn't create new vehicle connection");
  mavconn.set_protocol_version(mavlink::MavlinkVersion::V2);
  let connection = Arc::new(mavconn);
  
  thread::spawn({
      let connection = connection.clone();

      move || {
          loop {
              simulate_sensors_update(  &*connection,  &mut vehicle_state);
              thread::yield_now();
          }
      }
  });

    // receiving loop continues in this thread
    loop {
        match connection.recv() {
            Ok((_header, msg)) => {
                match msg.message_id() {
                  //TODO use constants instead
                  93 => continue, //HIL_ACTUATOR_CONTROLS
                  _ => println!("{:?}", msg)
                }
            },
            Err(e) => match e.kind() {
                std::io::ErrorKind::UnexpectedEof => {
                     //no more data coming
                    println!("UnexpectedEof: {:?}", e);
                    break;
                },
                std::io::ErrorKind::WouldBlock => {
                    //wait until something is available
                    //println!("WouldBlock: {:?}", e);
                },
                _ => {
                    println!("Err: {:?}", e);
                }
            },
        }
        thread::yield_now();
    }



}




