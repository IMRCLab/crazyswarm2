use crazyflie_lib::Crazyflie;
use rclrs::{Node, Service, Context, CreateBasicExecutor, SpinOptions, ServiceInfo};
use crazyflie_interfaces::srv::{Takeoff, Takeoff_Request, Takeoff_Response};
use std::sync::Arc;


struct CrazyflieROS {
    node: Node,
    cf: Arc<crazyflie_lib::Crazyflie>,
    service_takeoff: Service<Takeoff>,
}

impl CrazyflieROS {
    pub async fn new(node: Node, link_context: &crazyflie_link::LinkContext, uri: &str) -> Self {
        let cf = crazyflie_lib::Crazyflie::connect_from_uri(
                &link_context,
                uri).await.unwrap();
        println!("Connected!");
        let cfarc = Arc::<Crazyflie>::new(cf);

        let result = CrazyflieROS {
            node: node.clone(),
            cf: cfarc.clone(),
            service_takeoff: node.clone().create_async_service::<Takeoff, _>("/cfrs/takeoff", move |request: Takeoff_Request| {
                // handle_service(&cf, request)

                let seconds: f32 = (request.duration.sec as f32) + (request.duration.nanosec as f32) / 1e9;
                println!("takeoff(height={} m, duration={} s, group_mask={})",
                    request.height,
                    seconds,
                    request.group_mask);
                let cfarcclone = cfarc.clone();
                async move {
                    if let Err(e) = cfarcclone.high_level_commander.take_off(0.5, None, 2.0, None).await {
                        eprintln!("Take-off failed: {e}");
                    }

                    Takeoff_Response::default()
                }

            }).unwrap(),
        };

        
        let firmware_version = result.cf.platform.firmware_version().await.unwrap();
        let protocol_version = result.cf.platform.protocol_version().await.unwrap();
        println!(
            "Firmware version:     {} (protocol {})",
            firmware_version, protocol_version
        );

        let device_type = result.cf.platform.device_type_name().await.unwrap();
        println!("Device type:          {}", device_type);

        println!("Number of params var: {}", result.cf.param.names().len());
        println!("Number of log var:    {}", result.cf.log.names().len());


        result
    }
}


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

    let cfros = CrazyflieROS::new(node, &link_context, "radio://0/78/2M/E7E7E7E7E7").await;
    
    // let cf = crazyflie_lib::Crazyflie::connect_from_uri(
    //         &link_context,
    //         "radio://0/78/2M/E7E7E7E7E7",
    //     )
    //     .await?;

    // println!("Connected!");

    // let cfarc = Arc::<Crazyflie>::new(cf);
    // let cfarc2 = cfarc.clone();


    // let _server = node.create_async_service::<Takeoff, _>("/cfrs/takeoff", move |request: Takeoff_Request| {
    //     // handle_service(&cf, request)

    //     let seconds: f32 = (request.duration.sec as f32) + (request.duration.nanosec as f32) / 1e9;
    //     println!("takeoff(height={} m, duration={} s, group_mask={})",
    //         request.height,
    //         seconds,
    //         request.group_mask);
    //     let cfarcclone = cfarc.clone();
    //     async move {
    //         if let Err(e) = cfarcclone.high_level_commander.take_off(0.5, None, 2.0, None).await {
    //             eprintln!("Take-off failed: {e}");
    //         }

    //         Takeoff_Response::default()
    //     }

    // })?;


    // let firmware_version = cfarc2.platform.firmware_version().await?;
    // let protocol_version = cfarc2.platform.protocol_version().await?;
    // println!(
    //     "Firmware version:     {} (protocol {})",
    //     firmware_version, protocol_version
    // );

    // let device_type = cfarc2.platform.device_type_name().await?;
    // println!("Device type:          {}", device_type);

    // println!("Number of params var: {}", cfarc2.param.names().len());
    // println!("Number of log var:    {}", cfarc2.log.names().len());

    // cf.disconnect().await;


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

    drop(cfros);


    Ok(())
}
