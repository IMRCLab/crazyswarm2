use rclrs::{Context, CreateBasicExecutor, SpinOptions, ServiceInfo};
use crazyflie_interfaces::srv::{Takeoff, Takeoff_Request, Takeoff_Response};
// use std::sync::Arc;



//   void takeoff(const std::shared_ptr<Takeoff::Request> request,
//                std::shared_ptr<Takeoff::Response> response)
//   {
//     RCLCPP_INFO(logger_, "[%s] takeoff(height=%f m, duration=%f s, group_mask=%d)", 
//                 name_.c_str(),
//                 request->height,
//                 rclcpp::Duration(request->duration).seconds(),
//                 request->group_mask);
//     cf_.takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
//   }

fn handle_service(request: Takeoff_Request, info: ServiceInfo) -> Takeoff_Response {
    let timestamp = info
        .received_timestamp
        .map(|t| format!(" at [{t:?}]"))
        .unwrap_or(String::new());

    let seconds: f32 = (request.duration.sec as f32) + (request.duration.nanosec as f32) / 1e9;
    println!("request{timestamp}: takeoff(height={} m, duration={} s, group_mask={})",
        request.height,
        seconds,
        request.group_mask);
    Takeoff_Response { structure_needs_at_least_one_member: 0 }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("crazyflie_server_rs")?;

    let _server = node.create_service::<Takeoff, _>("/cfrs/takeoff", handle_service)?;


    // let broadcasts_num_repeats_ = node
    //     .declare_parameter("all.broadcasts.num_repeats")
    //     .default(15).mandatory().unwrap();


    // println!("{}", broadcasts_num_repeats_.get());

    // println!("{}", node.use_undeclared_parameters().get::<bool>("robots.cf231.enabled").unwrap());


    let link_context = crazyflie_link::LinkContext::new();

    let cf = crazyflie_lib::Crazyflie::connect_from_uri(
            &link_context,
            "radio://0/78/2M/E7E7E7E7E7",
        )
        .await?;

    println!("Connected!");

    let firmware_version = cf.platform.firmware_version().await?;
    let protocol_version = cf.platform.protocol_version().await?;
    println!(
        "Firmware version:     {} (protocol {})",
        firmware_version, protocol_version
    );

    let device_type = cf.platform.device_type_name().await?;
    println!("Device type:          {}", device_type);

    println!("Number of params var: {}", cf.param.names().len());
    println!("Number of log var:    {}", cf.log.names().len());

    cf.disconnect().await;


    // let storage = &node.use_undeclared_parameters().interface.parameter_map.lock().unwrap().storage;


    // let _subscription =
    //     node.create_subscription("greet", move |msg: example_interfaces::msg::String| {
    //         println!("{}, {}", greeting.get(), msg.data);
    //     })?;

    // println!(
    //     "Ready to provide a greeting. \
    //     \n\nTo see a greeting, try running\n \
    //     $ ros2 topic pub greet example_interfaces/msg/String \"data: Alice\"\
    //     \n\nTo change the kind of greeting, try running\n \
    //     $ ros2 param set parameter_demo greeting \"Guten tag\"\n"
    // );
    executor.spin(SpinOptions::default());

    Ok(())
}
