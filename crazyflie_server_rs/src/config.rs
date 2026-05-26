use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CrazyfliesConfig {
    #[serde(default)]
    pub fileversion: u32,
    
    #[serde(default)]
    pub robots: HashMap<String, RobotConfig>,
    
    #[serde(default)]
    pub robot_types: HashMap<String, RobotTypeConfig>,
    
    #[serde(default)]
    pub all: GlobalConfig,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RobotConfig {
    #[serde(default)]
    pub enabled: bool,
    
    #[serde(default)]
    pub uri: String,
    
    #[serde(default)]
    pub initial_position: Vec<f64>,
    
    #[serde(default)]
    pub r#type: String,
    
    #[serde(default)]
    pub firmware_params: HashMap<String, HashMap<String, serde_yaml::Value>>,
    
    #[serde(default)]
    pub firmware_logging: Option<FirmwareLogging>,
    
    #[serde(default)]
    pub reference_frame: Option<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RobotTypeConfig {
    #[serde(default)]
    pub motion_capture: MotionCaptureConfig,
    
    #[serde(default)]
    pub big_quad: bool,
    
    #[serde(default)]
    pub battery: BatteryConfig,
    
    #[serde(default)]
    pub firmware_params: HashMap<String, HashMap<String, serde_yaml::Value>>,
    
    #[serde(default)]
    pub firmware_logging: Option<FirmwareLogging>,
    
    #[serde(default)]
    pub reference_frame: Option<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct MotionCaptureConfig {
    #[serde(default)]
    pub tracking: String,
    
    #[serde(default)]
    pub marker: String,
    
    #[serde(default)]
    pub dynamics: String,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct BatteryConfig {
    #[serde(default)]
    pub voltage_warning: f32,
    
    #[serde(default)]
    pub voltage_critical: f32,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct FirmwareLogging {
    #[serde(default)]
    pub enabled: bool,
    
    #[serde(default)]
    pub default_topics: HashMap<String, TopicConfig>,
    
    #[serde(default)]
    pub custom_topics: HashMap<String, TopicConfig>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct TopicConfig {
    #[serde(default)]
    pub frequency: u32,
    
    #[serde(default)]
    pub vars: Vec<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct GlobalConfig {
    #[serde(default)]
    pub firmware_logging: Option<FirmwareLogging>,
    
    #[serde(default)]
    pub firmware_params: HashMap<String, HashMap<String, serde_yaml::Value>>,
    
    #[serde(default)]
    pub reference_frame: String,
    
    #[serde(default)]
    pub broadcasts: BroadcastsConfig,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct BroadcastsConfig {
    #[serde(default)]
    pub num_repeats: u32,
    
    #[serde(default)]
    pub delay_between_repeats_ms: u32,
}

impl CrazyfliesConfig {
    /// Load configuration from a YAML file
    pub fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let config: CrazyfliesConfig = serde_yaml::from_str(&contents)?;
        Ok(config)
    }
    
    /// Get all enabled robots with their configurations
    pub fn get_enabled_robots(&self) -> Vec<(String, &RobotConfig)> {
        self.robots
            .iter()
            .filter(|(_, robot)| robot.enabled)
            .map(|(name, robot)| (name.clone(), robot))
            .collect()
    }
    
    /// Get the effective logging configuration for a robot
    /// (merges global, type, and robot-specific settings)
    pub fn get_logging_config(&self, robot_name: &str) -> Option<FirmwareLogging> {
        let robot = self.robots.get(robot_name)?;
        
        // Start with global config
        let mut logging = self.all.firmware_logging.clone();
        
        // Merge robot type config if available
        if let Some(robot_type) = self.robot_types.get(&robot.r#type) {
            if let Some(type_logging) = &robot_type.firmware_logging {
                logging = Some(merge_logging_config(logging, type_logging.clone()));
            }
        }
        
        // Merge robot-specific config
        if let Some(robot_logging) = &robot.firmware_logging {
            logging = Some(merge_logging_config(logging, robot_logging.clone()));
        }
        
        logging
    }
    
    /// Get the effective reference frame for a robot
    pub fn get_reference_frame(&self, robot_name: &str) -> String {
        let robot = self.robots.get(robot_name);
        
        // Robot-specific frame has highest priority
        if let Some(robot) = robot {
            if let Some(ref frame) = robot.reference_frame {
                return frame.clone();
            }
            
            // Then check robot type
            if let Some(robot_type) = self.robot_types.get(&robot.r#type) {
                if let Some(ref frame) = robot_type.reference_frame {
                    return frame.clone();
                }
            }
        }
        
        // Fall back to global default
        self.all.reference_frame.clone()
    }
}

/// Helper to merge two logging configurations
fn merge_logging_config(
    base: Option<FirmwareLogging>,
    overlay: FirmwareLogging,
) -> FirmwareLogging {
    match base {
        None => overlay,
        Some(mut base) => {
            // Overlay enabled status
            base.enabled = overlay.enabled;
            
            // Merge default topics
            for (name, config) in overlay.default_topics {
                base.default_topics.insert(name, config);
            }
            
            // Merge custom topics
            for (name, config) in overlay.custom_topics {
                base.custom_topics.insert(name, config);
            }
            
            base
        }
    }
}
