/// Apply firmware parameters from configuration
pub async fn apply_firmware_params(
    cf: &crazyflie_lib::Crazyflie,
    params: &std::collections::HashMap<String, std::collections::HashMap<String, serde_yaml::Value>>,
    cf_name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    use crazyflie_lib::Value;
    
    for (group, group_params) in params {
        for (param_name, value) in group_params {
            let full_name = format!("{}.{}", group, param_name);
            
            // Convert serde_yaml::Value to crazyflie_lib::Value
            let cf_value = match value {
                serde_yaml::Value::Number(n) => {
                    if let Some(i) = n.as_i64() {
                        if i >= 0 && i <= 255 {
                            Value::U8(i as u8)
                        } else if i >= i8::MIN as i64 && i <= i8::MAX as i64 {
                            Value::I8(i as i8)
                        } else if i >= 0 && i <= u16::MAX as i64 {
                            Value::U16(i as u16)
                        } else if i >= i16::MIN as i64 && i <= i16::MAX as i64 {
                            Value::I16(i as i16)
                        } else if i >= 0 && i <= u32::MAX as i64 {
                            Value::U32(i as u32)
                        } else {
                            Value::I32(i as i32)
                        }
                    } else if let Some(f) = n.as_f64() {
                        Value::F32(f as f32)
                    } else {
                        continue;
                    }
                }
                serde_yaml::Value::Bool(b) => Value::U8(if *b { 1 } else { 0 }),
                _ => {
                    eprintln!("[{}] Unsupported parameter value type for {}", cf_name, full_name);
                    continue;
                }
            };
            
            match cf.param.set(&full_name, cf_value).await {
                Ok(_) => {
                    println!("[{}] Set parameter {} = {:?}", cf_name, full_name, value);
                }
                Err(e) => {
                    eprintln!("[{}] Failed to set parameter {}: {}", cf_name, full_name, e);
                }
            }
        }
    }
    
    Ok(())
}
