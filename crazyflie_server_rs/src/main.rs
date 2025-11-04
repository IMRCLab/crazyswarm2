use rclrs::{Context, CreateBasicExecutor, SpinOptions};
// use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("crazyflie_server_rs")?;

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
