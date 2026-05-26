use crazyflie_lib::Crazyflie;
use rclrs::{Node, Service, Publisher, Subscription, Context, CreateBasicExecutor, SpinOptions};
use crazyflie_interfaces::{
    srv::{
        Takeoff, Takeoff_Request, Takeoff_Response,
        Land, Land_Request, Land_Response,
        GoTo, GoTo_Request, GoTo_Response,
        Arm, Arm_Request, Arm_Response,
        StartTrajectory, StartTrajectory_Request, StartTrajectory_Response,
        UploadTrajectory, UploadTrajectory_Request, UploadTrajectory_Response,
        NotifySetpointsStop, NotifySetpointsStop_Request, NotifySetpointsStop_Response,
    },
    msg::{
        Position,
        VelocityWorld,
        Hover,
        FullState,
        Status,
    },
};
use std_srvs::srv::{Empty, Empty_Request, Empty_Response};
use geometry_msgs::msg::{Twist, PoseStamped, Quaternion};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::Odometry;
use std_msgs::msg::String as StringMsg;
use std::sync::Arc;

mod config;
use config::CrazyfliesConfig;

mod params;
mod log_helpers;
use log_helpers::LogDataExt;


struct CrazyflieROS {
    _node: Node,
    cf: Arc<crazyflie_lib::Crazyflie>,
    cf_name: String,
    // Services
    _service_emergency: Service<Empty>,
    _service_arm: Service<Arm>,
    _service_takeoff: Service<Takeoff>,
    _service_land: Service<Land>,
    _service_go_to: Service<GoTo>,
    _service_start_trajectory: Service<StartTrajectory>,
    _service_upload_trajectory: Service<UploadTrajectory>,
    _service_notify_setpoints_stop: Service<NotifySetpointsStop>,
    // Subscriptions
    _sub_cmd_vel_legacy: Subscription<Twist>,
    _sub_cmd_position: Subscription<Position>,
    _sub_cmd_velocity_world: Subscription<VelocityWorld>,
    _sub_cmd_hover: Subscription<Hover>,
    _sub_cmd_full_state: Subscription<FullState>,
    // Publishers
    _pub_robot_description: Publisher<StringMsg>,
    _pub_pose: Option<Publisher<PoseStamped>>,
    _pub_scan: Option<Publisher<LaserScan>>,
    _pub_odom: Option<Publisher<Odometry>>,
    _pub_status: Option<Publisher<Status>>,
}

impl CrazyflieROS {
    pub async fn new(
        node: Node, 
        link_context: &crazyflie_link::LinkContext, 
        uri: &str,
        cf_name: String,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        // Connect to Crazyflie
        let cf = crazyflie_lib::Crazyflie::connect_from_uri(
            &link_context,
            uri,
        ).await?;
        
        println!("[{}] Connected!", cf_name);
        let cfarc = Arc::<Crazyflie>::new(cf);

        // Display firmware info
        let firmware_version = cfarc.platform.firmware_version().await?;
        let protocol_version = cfarc.platform.protocol_version().await?;
        println!(
            "[{}] Firmware version: {} (protocol {})",
            cf_name, firmware_version, protocol_version
        );

        let device_type = cfarc.platform.device_type_name().await?;
        println!("[{}] Device type: {}", cf_name, device_type);
        println!("[{}] Number of params: {}", cf_name, cfarc.param.names().len());
        println!("[{}] Number of log vars: {}", cf_name, cfarc.log.names().len());

        // Create services
        let cf_emergency = cfarc.clone();
        let service_emergency = node.create_async_service::<Empty, _>(
            &format!("{}/emergency", cf_name),
            move |_request: Empty_Request| {
                let cf = cf_emergency.clone();
                async move {
                    println!("[Emergency] Stopping motors");
                    if let Err(e) = cf.commander.setpoint_stop().await {
                        eprintln!("Emergency stop failed: {e}");
                    }
                    Empty_Response::default()
                }
            },
        )?;

        let cf_arm = cfarc.clone();
        let service_arm = node.create_async_service::<Arm, _>(
            &format!("{}/arm", cf_name),
            move |request: Arm_Request| {
                let cf = cf_arm.clone();
                async move {
                    println!("[Arm] arm={}", request.arm);
                    // Set the arm parameter via firmware param
                    // Note: stabilizer.stop = 0 means armed, 1 means stopped
                    let value = if request.arm {
                        crazyflie_lib::Value::U8(0)  // Armed
                    } else {
                        crazyflie_lib::Value::U8(1)  // Stopped
                    };
                    if let Err(e) = cf.param.set("stabilizer.stop", value).await {
                        eprintln!("Failed to set arm parameter: {}", e);
                    }
                    Arm_Response::default()
                }
            },
        )?;

        let cf_takeoff = cfarc.clone();
        let name_takeoff = cf_name.clone();
        let service_takeoff = node.create_async_service::<Takeoff, _>(
            &format!("{}/takeoff", cf_name),
            move |request: Takeoff_Request| {
                let cf = cf_takeoff.clone();
                let name = name_takeoff.clone();
                async move {
                    let duration = request.duration.sec as f32 + request.duration.nanosec as f32 / 1e9;
                    println!(
                        "[{}] takeoff(height={} m, duration={} s, group_mask={})",
                        name, request.height, duration, request.group_mask
                    );
                    if let Err(e) = cf.high_level_commander
                        .take_off(request.height, None, duration, None)
                        .await 
                    {
                        eprintln!("[{}] Takeoff failed: {e}", name);
                    }
                    Takeoff_Response::default()
                }
            },
        )?;

        let cf_land = cfarc.clone();
        let name_land = cf_name.clone();
        let service_land = node.create_async_service::<Land, _>(
            &format!("{}/land", cf_name),
            move |request: Land_Request| {
                let cf = cf_land.clone();
                let name = name_land.clone();
                async move {
                    let duration = request.duration.sec as f32 + request.duration.nanosec as f32 / 1e9;
                    println!(
                        "[{}] land(height={} m, duration={} s, group_mask={})",
                        name, request.height, duration, request.group_mask
                    );
                    if let Err(e) = cf.high_level_commander
                        .land(request.height, None, duration, None)
                        .await 
                    {
                        eprintln!("[{}] Land failed: {e}", name);
                    }
                    Land_Response::default()
                }
            },
        )?;

        let cf_goto = cfarc.clone();
        let name_goto = cf_name.clone();
        let service_go_to = node.create_async_service::<GoTo, _>(
            &format!("{}/go_to", cf_name),
            move |request: GoTo_Request| {
                let cf = cf_goto.clone();
                let name = name_goto.clone();
                async move {
                    let duration = request.duration.sec as f32 + request.duration.nanosec as f32 / 1e9;
                    println!(
                        "[{}] go_to(position=({}, {}, {}) m, yaw={} deg, duration={} s, relative={}, group_mask={})",
                        name,
                        request.goal.x, request.goal.y, request.goal.z,
                        request.yaw, duration, request.relative, request.group_mask
                    );
                    if let Err(e) = cf.high_level_commander
                        .go_to(
                            request.goal.x as f32, 
                            request.goal.y as f32, 
                            request.goal.z as f32,
                            request.yaw,
                            duration,
                            request.relative,
                            false,
                            None
                        )
                        .await 
                    {
                        eprintln!("[{}] Go to failed: {e}", name);
                    }
                    GoTo_Response::default()
                }
            },
        )?;

        let cf_start_traj = cfarc.clone();
        let name_start_traj = cf_name.clone();
        let service_start_trajectory = node.create_async_service::<StartTrajectory, _>(
            &format!("{}/start_trajectory", cf_name),
            move |request: StartTrajectory_Request| {
                let cf = cf_start_traj.clone();
                let name = name_start_traj.clone();
                async move {
                    println!(
                        "[{}] start_trajectory(id={}, timescale={}, relative={}, reversed={}, group_mask={})",
                        name,
                        request.trajectory_id, request.timescale, 
                        request.relative, request.reversed, request.group_mask
                    );
                    if let Err(e) = cf.high_level_commander
                        .start_trajectory(
                            request.trajectory_id,
                            request.timescale,
                            request.relative,
                            false,
                            request.reversed,
                            None
                        )
                        .await 
                    {
                        eprintln!("[{}] Start trajectory failed: {e}", name);
                    }
                    StartTrajectory_Response::default()
                }
            },
        )?;

        let cf_upload_traj = cfarc.clone();
        let name_upload_traj = cf_name.clone();
        let service_upload_trajectory = node.create_async_service::<UploadTrajectory, _>(
            &format!("{}/upload_trajectory", cf_name),
            move |request: UploadTrajectory_Request| {
                let _cf = cf_upload_traj.clone();
                let name = name_upload_traj.clone();
                async move {
                    println!(
                        "[{}] upload_trajectory(id={}, offset={}, length={})",
                        name,
                        request.trajectory_id, request.piece_offset, request.pieces.len()
                    );
                    // Trajectory upload using memory subsystem
                    // Note: This is a stub - full implementation requires:
                    // 1. Getting trajectory memory handle from cf.memory
                    // 2. Converting TrajectoryPolynomialPiece to Poly4D format
                    // 3. Writing to trajectory memory slot
                    // The crazyflie-lib API for this is:
                    //   let traj_mem = cf.memory.trajectory_memories().await?;
                    //   traj_mem.write(id, &poly4d_segments).await?;
                    eprintln!("[{}] Trajectory upload: Converting {} pieces to Poly4D format", name, request.pieces.len());
                    eprintln!("[{}] Trajectory upload not fully implemented - requires memory subsystem integration", name);
                    // TODO: Implement full conversion and upload
                    UploadTrajectory_Response::default()
                }
            },
        )?;

        let cf_notify_stop = cfarc.clone();
        let name_notify_stop = cf_name.clone();
        let service_notify_setpoints_stop = node.create_async_service::<NotifySetpointsStop, _>(
            &format!("{}/notify_setpoints_stop", cf_name),
            move |request: NotifySetpointsStop_Request| {
                let cf = cf_notify_stop.clone();
                let name = name_notify_stop.clone();
                async move {
                    println!("[{}] notify_setpoints_stop(remain_valid_ms={})", name, request.remain_valid_millisecs);
                    if let Err(e) = cf.commander
                        .notify_setpoint_stop(request.remain_valid_millisecs)
                        .await 
                    {
                        eprintln!("[{}] Notify setpoints stop failed: {e}", name);
                    }
                    NotifySetpointsStop_Response::default()
                }
            },
        )?;

        // Create subscriptions for command topics
        let cf_cmd_vel = cfarc.clone();
        let sub_cmd_vel_legacy = node.create_subscription(
            &format!("{}/cmd_vel_legacy", cf_name),
            move |msg: Twist| {
                let cf = cf_cmd_vel.clone();
                tokio::spawn(async move {
                    let roll = msg.linear.y;
                    let pitch = -msg.linear.x;
                    let yawrate = msg.angular.z as f32;
                    let thrust = msg.linear.z.min(60000.0).max(0.0) as u16;
                    if let Err(e) = cf.commander.setpoint_rpyt(roll as f32, pitch as f32, yawrate, thrust).await {
                        eprintln!("cmd_vel_legacy failed: {e}");
                    }
                });
            },
        )?;

        let cf_cmd_pos = cfarc.clone();
        let sub_cmd_position = node.create_subscription(
            &format!("{}/cmd_position", cf_name),
            move |msg: Position| {
                let cf = cf_cmd_pos.clone();
                tokio::spawn(async move {
                    if let Err(e) = cf.commander
                        .setpoint_position(msg.x, msg.y, msg.z, msg.yaw)
                        .await 
                    {
                        eprintln!("cmd_position failed: {e}");
                    }
                });
            },
        )?;

        let cf_cmd_vel_world = cfarc.clone();
        let sub_cmd_velocity_world = node.create_subscription(
            &format!("{}/cmd_velocity_world", cf_name),
            move |msg: VelocityWorld| {
                let cf = cf_cmd_vel_world.clone();
                tokio::spawn(async move {
                    if let Err(e) = cf.commander
                        .setpoint_velocity_world(
                            msg.vel.x as f32, 
                            msg.vel.y as f32, 
                            msg.vel.z as f32, 
                            msg.yaw_rate
                        )
                        .await 
                    {
                        eprintln!("cmd_velocity_world failed: {e}");
                    }
                });
            },
        )?;

        let cf_cmd_hover = cfarc.clone();
        let sub_cmd_hover = node.create_subscription(
            &format!("{}/cmd_hover", cf_name),
            move |msg: Hover| {
                let cf = cf_cmd_hover.clone();
                tokio::spawn(async move {
                    // Convert yaw_rate from rad/s to deg/s and negate
                    let yawrate_deg = -msg.yaw_rate.to_degrees();
                    if let Err(e) = cf.commander
                        .setpoint_hover(msg.vx, msg.vy, yawrate_deg, msg.z_distance)
                        .await 
                    {
                        eprintln!("cmd_hover failed: {e}");
                    }
                });
            },
        )?;

        let cf_cmd_full = cfarc.clone();
        let sub_cmd_full_state = node.create_subscription(
            &format!("{}/cmd_full_state", cf_name),
            move |msg: FullState| {
                let cf = cf_cmd_full.clone();
                tokio::spawn(async move {
                    // Full state setpoint: position + velocity + acceleration + attitude
                    // crazyflie-lib doesn't have a direct full_state method, but we can use
                    // setpoint_position as the best approximation for now
                    // In the future, this could be extended to send a CRTP packet with all data
                    if let Err(e) = cf.commander
                        .setpoint_position(
                            msg.pose.position.x as f32,
                            msg.pose.position.y as f32,
                            msg.pose.position.z as f32,
                            msg.twist.angular.z as f32, // yaw from angular.z
                        )
                        .await 
                    {
                        eprintln!("cmd_full_state (using position) failed: {e}");
                    }
                    // Note: Velocity and acceleration from FullState are currently ignored
                    // as crazyflie-lib doesn't expose a combined full-state setpoint
                });
            },
        )?;

        // Create robot description publisher
        let pub_robot_description = node.create_publisher(
            &format!("{}/robot_description", cf_name),
        )?;
        
        // Publish a basic URDF for the Crazyflie
        let urdf = format!(
            r#"<?xml version="1.0"?>
<robot name="{name}">
  <link name="{name}/base_link">
    <inertial>
      <mass value="0.027"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.65e-05" ixy="0.0" ixz="0.0" iyy="1.65e-05" iyz="0.0" izz="2.92e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.01"/>
      </geometry>
    </visual>
  </link>
</robot>"#,
            name = cf_name
        );
        
        let urdf_msg = StringMsg { data: urdf };
        if let Err(e) = pub_robot_description.publish(urdf_msg) {
            eprintln!("[{}] Failed to publish robot description: {}", cf_name, e);
        }
        
        // Create logging publishers (initially None, will be created based on config)
        let pub_pose = None;
        let pub_scan = None;
        let pub_odom = None;
        let pub_status = None;

        Ok(CrazyflieROS {
            _node: node,
            cf: cfarc,
            cf_name,
            _service_emergency: service_emergency,
            _service_arm: service_arm,
            _service_takeoff: service_takeoff,
            _service_land: service_land,
            _service_go_to: service_go_to,
            _service_start_trajectory: service_start_trajectory,
            _service_upload_trajectory: service_upload_trajectory,
            _service_notify_setpoints_stop: service_notify_setpoints_stop,
            _sub_cmd_vel_legacy: sub_cmd_vel_legacy,
            _sub_cmd_position: sub_cmd_position,
            _sub_cmd_velocity_world: sub_cmd_velocity_world,
            _sub_cmd_hover: sub_cmd_hover,
            _sub_cmd_full_state: sub_cmd_full_state,
            _pub_robot_description: pub_robot_description,
            _pub_pose: pub_pose,
            _pub_scan: pub_scan,
            _pub_odom: pub_odom,
            _pub_status: pub_status,
        })
    }

    /// Initialize logging for pose data
    /// This creates a log block that reads state estimation and attitude data
    pub async fn init_pose_logging(
        &mut self,
        node: &Node,
        frequency_hz: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pub_pose: Publisher<PoseStamped> = node.create_publisher(
            &format!("{}/pose", self.cf_name),
        )?;

        // Create log block with crazyflie-lib
        let mut block = self.cf.log.create_block().await?;
        block.add_variable("stateEstimate.x").await?;
        block.add_variable("stateEstimate.y").await?;
        block.add_variable("stateEstimate.z").await?;
        block.add_variable("stabilizer.roll").await?;
        block.add_variable("stabilizer.pitch").await?;
        block.add_variable("stabilizer.yaw").await?;
        
        let period = crazyflie_lib::subsystems::log::LogPeriod::from_millis(
            (1000 / frequency_hz) as u64
        )?;
        let mut stream = block.start(period).await?;
        
        // Spawn task to handle log data
        let cf_name = self.cf_name.clone();
        let pub_pose_clone = pub_pose.clone();
        tokio::spawn(async move {
            while let Ok(data) = stream.next().await {
                let mut pose_msg = PoseStamped::default();
                
                // Position
                if let Some(x) = data.get_f32("stateEstimate.x") {
                    pose_msg.pose.position.x = x as f64;
                }
                if let Some(y) = data.get_f32("stateEstimate.y") {
                    pose_msg.pose.position.y = y as f64;
                }
                if let Some(z) = data.get_f32("stateEstimate.z") {
                    pose_msg.pose.position.z = z as f64;
                }
                
                // Orientation (convert Euler to Quaternion)
                let roll = data.get_f32("stabilizer.roll").unwrap_or(0.0) * std::f32::consts::PI / 180.0;
                let pitch = data.get_f32("stabilizer.pitch").unwrap_or(0.0) * std::f32::consts::PI / 180.0;
                let yaw = data.get_f32("stabilizer.yaw").unwrap_or(0.0) * std::f32::consts::PI / 180.0;
                
                pose_msg.pose.orientation = Self::euler_to_quaternion(
                    roll as f64, pitch as f64, yaw as f64
                );
                
                // Publish
                if let Err(e) = pub_pose_clone.publish(pose_msg) {
                    eprintln!("[{}] Failed to publish pose: {}", cf_name, e);
                }
            }
        });
        
        self._pub_pose = Some(pub_pose);
        println!("[{}] Pose logging initialized at {} Hz", self.cf_name, frequency_hz);
        Ok(())
    }

    /// Initialize logging for odometry data
    pub async fn init_odom_logging(
        &mut self,
        node: &Node,
        frequency_hz: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pub_odom: Publisher<Odometry> = node.create_publisher(
            &format!("{}/odom", self.cf_name),
        )?;

        // Create log block with crazyflie-lib
        let mut block = self.cf.log.create_block().await?;
        block.add_variable("stateEstimate.x").await?;
        block.add_variable("stateEstimate.y").await?;
        block.add_variable("stateEstimate.z").await?;
        block.add_variable("stabilizer.roll").await?;
        block.add_variable("stabilizer.pitch").await?;
        block.add_variable("stabilizer.yaw").await?;
        block.add_variable("kalman.statePX").await?;
        block.add_variable("kalman.statePY").await?;
        block.add_variable("kalman.statePZ").await?;
        block.add_variable("gyro.x").await?;
        block.add_variable("gyro.y").await?;
        block.add_variable("gyro.z").await?;
        
        let period = crazyflie_lib::subsystems::log::LogPeriod::from_millis(
            (1000 / frequency_hz) as u64
        )?;
        let mut stream = block.start(period).await?;
        
        // Spawn task to handle log data
        let cf_name = self.cf_name.clone();
        let pub_odom_clone = pub_odom.clone();
        tokio::spawn(async move {
            while let Ok(data) = stream.next().await {
                let mut odom_msg = Odometry::default();
                
                // Position
                if let Some(x) = data.get_f32("stateEstimate.x") {
                    odom_msg.pose.pose.position.x = x as f64;
                }
                if let Some(y) = data.get_f32("stateEstimate.y") {
                    odom_msg.pose.pose.position.y = y as f64;
                }
                if let Some(z) = data.get_f32("stateEstimate.z") {
                    odom_msg.pose.pose.position.z = z as f64;
                }
                
                // Orientation
                let roll = data.get_f32("stabilizer.roll").unwrap_or(0.0) * std::f32::consts::PI / 180.0;
                let pitch = data.get_f32("stabilizer.pitch").unwrap_or(0.0) * std::f32::consts::PI / 180.0;
                let yaw = data.get_f32("stabilizer.yaw").unwrap_or(0.0) * std::f32::consts::PI / 180.0;
                
                odom_msg.pose.pose.orientation = Self::euler_to_quaternion(
                    roll as f64, pitch as f64, yaw as f64
                );
                
                // Velocity (from Kalman filter state)
                if let Some(vx) = data.get_f32("kalman.statePX") {
                    odom_msg.twist.twist.linear.x = vx as f64;
                }
                if let Some(vy) = data.get_f32("kalman.statePY") {
                    odom_msg.twist.twist.linear.z = vy as f64;
                }
                if let Some(vz) = data.get_f32("kalman.statePZ") {
                    odom_msg.twist.twist.linear.z = vz as f64;
                }
                
                // Angular velocity (from gyro)
                if let Some(gx) = data.get_f32("gyro.x") {
                    odom_msg.twist.twist.angular.x = (gx * std::f32::consts::PI / 180.0) as f64;
                }
                if let Some(gy) = data.get_f32("gyro.y") {
                    odom_msg.twist.twist.angular.y = (gy * std::f32::consts::PI / 180.0) as f64;
                }
                if let Some(gz) = data.get_f32("gyro.z") {
                    odom_msg.twist.twist.angular.z = (gz * std::f32::consts::PI / 180.0) as f64;
                }
                
                // Publish
                if let Err(e) = pub_odom_clone.publish(odom_msg) {
                    eprintln!("[{}] Failed to publish odom: {}", cf_name, e);
                }
            }
        });
        
        self._pub_odom = Some(pub_odom);
        println!("[{}] Odom logging initialized at {} Hz", self.cf_name, frequency_hz);
        Ok(())
    }

    /// Initialize logging for multiranger scan data
    pub async fn init_scan_logging(
        &mut self,
        node: &Node,
        frequency_hz: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pub_scan: Publisher<LaserScan> = node.create_publisher(
            &format!("{}/scan", self.cf_name),
        )?;

        // Create log block with crazyflie-lib
        let mut block = self.cf.log.create_block().await?;
        block.add_variable("range.front").await?;
        block.add_variable("range.left").await?;
        block.add_variable("range.back").await?;
        block.add_variable("range.right").await?;
        
        let period = crazyflie_lib::subsystems::log::LogPeriod::from_millis(
            (1000 / frequency_hz) as u64
        )?;
        let mut stream = block.start(period).await?;
        
        // Spawn task to handle log data
        let cf_name = self.cf_name.clone();
        let pub_scan_clone = pub_scan.clone();
        tokio::spawn(async move {
            while let Ok(data) = stream.next().await {
                let mut scan_msg = LaserScan::default();
                
                // Setup scan parameters (4 ranges: front, left, back, right)
                scan_msg.angle_min = -std::f32::consts::PI;
                scan_msg.angle_max = std::f32::consts::PI;
                scan_msg.angle_increment = std::f32::consts::PI / 2.0;
                scan_msg.range_min = 0.01;
                scan_msg.range_max = 3.5;
                
                // Get range values (in mm, convert to m)
                let front = data.get_u16("range.front").unwrap_or(0) as f32 / 1000.0;
                let left = data.get_u16("range.left").unwrap_or(0) as f32 / 1000.0;
                let back = data.get_u16("range.back").unwrap_or(0) as f32 / 1000.0;
                let right = data.get_u16("range.right").unwrap_or(0) as f32 / 1000.0;
                
                // Arrange in order: front, left, back, right
                scan_msg.ranges = vec![front, left, back, right];
                
                // Publish
                if let Err(e) = pub_scan_clone.publish(scan_msg) {
                    eprintln!("[{}] Failed to publish scan: {}", cf_name, e);
                }
            }
        });
        
        self._pub_scan = Some(pub_scan);
        println!("[{}] Scan logging initialized at {} Hz", self.cf_name, frequency_hz);
        Ok(())
    }

    /// Initialize logging for status data
    pub async fn init_status_logging(
        &mut self,
        node: &Node,
        frequency_hz: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pub_status: Publisher<Status> = node.create_publisher(
            &format!("{}/status", self.cf_name),
        )?;

        // Create log block with crazyflie-lib
        let mut block = self.cf.log.create_block().await?;
        block.add_variable("supervisor.info").await?;
        block.add_variable("pm.vbatMV").await?;
        block.add_variable("pm.state").await?;
        
        let period = crazyflie_lib::subsystems::log::LogPeriod::from_millis(
            (1000 / frequency_hz) as u64
        )?;
        let mut stream = block.start(period).await?;
        
        // Spawn task to handle log data
        let cf_name = self.cf_name.clone();
        let pub_status_clone = pub_status.clone();
        tokio::spawn(async move {
            while let Ok(data) = stream.next().await {
                let mut status_msg = Status::default();
                
                // Supervisor info (bitfield)
                if let Some(info) = data.get_u16("supervisor.info") {
                    status_msg.supervisor_info = info;
                }
                
                // Battery voltage (mV to V)
                if let Some(vbat) = data.get_u16("pm.vbatMV") {
                    status_msg.battery_voltage = vbat as f32 / 1000.0;
                }
                
                // PM state
                if let Some(state) = data.get_u8("pm.state") {
                    status_msg.pm_state = state;
                }
                
                // Publish
                if let Err(e) = pub_status_clone.publish(status_msg) {
                    eprintln!("[{}] Failed to publish status: {}", cf_name, e);
                }
            }
        });
        
        self._pub_status = Some(pub_status);
        println!("[{}] Status logging initialized at {} Hz", self.cf_name, frequency_hz);
        Ok(())
    }

    /// Helper function to convert Euler angles (roll, pitch, yaw) to quaternion
    fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();

        Quaternion {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
}


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("crazyflie_server_rs")?;

    println!("==================================================");
    println!("  Crazyflie Server (Rust Implementation)");
    println!("  Based on crazyflie-lib-rs");
    println!("==================================================");

    // Load configuration from YAML file
    // Default path follows the same convention as the Python server
    let config_path = std::env::var("CRAZYFLIE_CONFIG_PATH")
        .unwrap_or_else(|_| {
            // Try to use ROS package share directory
            let default_path = "/home/kimbe/dev/ros_workspaces/crazyswarm2/src/crazyswarm2/crazyflie/config/crazyflies.yaml";
            println!("Using default config path: {}", default_path);
            default_path.to_string()
        });

    let config = match CrazyfliesConfig::from_file(&config_path) {
        Ok(cfg) => {
            println!("✓ Loaded configuration from: {}", config_path);
            println!("  File version: {}", cfg.fileversion);
            cfg
        }
        Err(e) => {
            eprintln!("✗ Failed to load configuration from {}: {}", config_path, e);
            eprintln!("\nFalling back to hardcoded configuration...");
            
            // Create a minimal default config
            CrazyfliesConfig {
                fileversion: 3,
                robots: std::collections::HashMap::new(),
                robot_types: std::collections::HashMap::new(),
                all: config::GlobalConfig::default(),
            }
        }
    };

    let link_context = crazyflie_link::LinkContext::new();
    let mut crazyflies: Vec<CrazyflieROS> = Vec::new();

    // Get enabled robots from configuration
    let enabled_robots = config.get_enabled_robots();
    
    if enabled_robots.is_empty() {
        eprintln!("\n⚠ No enabled robots found in configuration!");
        eprintln!("Please check your crazyflies.yaml file.");
        eprintln!("\nExample configuration:");
        eprintln!("robots:");
        eprintln!("  cf1:");
        eprintln!("    enabled: true");
        eprintln!("    uri: radio://0/80/2M/E7E7E7E7E7");
        eprintln!("    type: cf21");
        return Err("No enabled robots in configuration".into());
    }

    println!("\n📋 Found {} enabled robot(s) in configuration:", enabled_robots.len());
    for (name, robot) in &enabled_robots {
        println!("  - {} ({})", name, robot.uri);
    }

    // Connect to each enabled robot
    for (cf_name, robot_config) in enabled_robots {
        println!("\n[{}] Connecting to {}...", cf_name, robot_config.uri);
        
        match CrazyflieROS::new(
            node.clone(),
            &link_context,
            &robot_config.uri,
            cf_name.clone(),
        ).await {
            Ok(mut cfros) => {
                // Apply firmware parameters from configuration
                // Merge global, type-specific, and robot-specific parameters
                let mut all_params = config.all.firmware_params.clone();
                
                // Merge type-specific params
                if let Some(robot_type) = config.robot_types.get(&robot_config.r#type) {
                    for (group, params) in &robot_type.firmware_params {
                        all_params.entry(group.clone())
                            .or_insert_with(std::collections::HashMap::new)
                            .extend(params.clone());
                    }
                }
                
                // Merge robot-specific params (highest priority)
                for (group, params) in &robot_config.firmware_params {
                    all_params.entry(group.clone())
                        .or_insert_with(std::collections::HashMap::new)
                        .extend(params.clone());
                }
                
                // Apply all parameters
                if !all_params.is_empty() {
                    println!("[{}] Applying {} firmware parameter group(s)...", cf_name, all_params.len());
                    if let Err(e) = params::apply_firmware_params(&cfros.cf, &all_params, &cf_name).await {
                        eprintln!("[{}] Failed to apply some firmware parameters: {}", cf_name, e);
                    }
                }
                
                // Get logging configuration for this robot
                if let Some(logging_config) = config.get_logging_config(&cf_name) {
                    if logging_config.enabled {
                        // Initialize default topics
                        for (topic_name, topic_config) in &logging_config.default_topics {
                            match topic_name.as_str() {
                                "pose" => {
                                    if let Err(e) = cfros.init_pose_logging(&node, topic_config.frequency).await {
                                        eprintln!("[{}] Failed to init pose logging: {}", cf_name, e);
                                    }
                                }
                                "status" => {
                                    if let Err(e) = cfros.init_status_logging(&node, topic_config.frequency).await {
                                        eprintln!("[{}] Failed to init status logging: {}", cf_name, e);
                                    }
                                }
                                "odom" => {
                                    if let Err(e) = cfros.init_odom_logging(&node, topic_config.frequency).await {
                                        eprintln!("[{}] Failed to init odom logging: {}", cf_name, e);
                                    }
                                }
                                "scan" => {
                                    if let Err(e) = cfros.init_scan_logging(&node, topic_config.frequency).await {
                                        eprintln!("[{}] Failed to init scan logging: {}", cf_name, e);
                                    }
                                }
                                _ => {
                                    println!("[{}] Unknown default topic: {}", cf_name, topic_name);
                                }
                            }
                        }
                        
                        // TODO: Initialize custom topics
                        if !logging_config.custom_topics.is_empty() {
                            println!("[{}] Custom topics configured but not yet implemented", cf_name);
                        }
                    }
                }
                
                crazyflies.push(cfros);
                println!("[{}] ✓ Fully initialized!", cf_name);
            }
            Err(e) => {
                eprintln!("[{}] ✗ Failed to connect: {}", cf_name, e);
                eprintln!("Skipping this Crazyflie and continuing...");
            }
        }
    }

    if crazyflies.is_empty() {
        eprintln!("\n✗ Error: No Crazyflies connected!");
        eprintln!("Check URIs and ensure Crazyflies are powered on.");
        return Err("No Crazyflies available".into());
    }

    println!("\n==================================================");
    println!("  ✓ {} Crazyflie(s) ready!", crazyflies.len());
    println!("  All services and subscriptions initialized.");
    println!("==================================================\n");

    println!("Services available per Crazyflie:");
    println!("  - <name>/emergency");
    println!("  - <name>/arm");
    println!("  - <name>/takeoff");
    println!("  - <name>/land");
    println!("  - <name>/go_to");
    println!("  - <name>/start_trajectory");
    println!("  - <name>/upload_trajectory");
    println!("  - <name>/notify_setpoints_stop");
    println!("\nSubscriptions available per Crazyflie:");
    println!("  - <name>/cmd_vel_legacy");
    println!("  - <name>/cmd_position");
    println!("  - <name>/cmd_velocity_world");
    println!("  - <name>/cmd_hover");
    println!("  - <name>/cmd_full_state");
    println!("\nPublishers available per Crazyflie:");
    println!("  - <name>/pose");
    println!("  - <name>/odom");
    println!("  - <name>/scan");
    println!("  - <name>/status");
    println!("  - <name>/robot_description\n");

    executor.spin(SpinOptions::default());

    // Cleanup
    println!("\nShutting down Crazyflie Server...");
    drop(crazyflies);
    println!("Shutdown complete.");

    Ok(())
}
