use crazyflie_lib::{Value, subsystems::log::LogData};

/// Helper trait to get typed values from LogData
pub trait LogDataExt {
    fn get_f32(&self, name: &str) -> Option<f32>;
    fn get_u16(&self, name: &str) -> Option<u16>;
    fn get_u8(&self, name: &str) -> Option<u8>;
    fn get_i32(&self, name: &str) -> Option<i32>;
}

impl LogDataExt for LogData {
    fn get_f32(&self, name: &str) -> Option<f32> {
        match self.data.get(name)? {
            Value::F32(v) => Some(*v),
            Value::F64(v) => Some(*v as f32),
            Value::F16(v) => Some(f32::from(*v)),
            Value::I64(v) => Some(*v as f32),
            Value::U64(v) => Some(*v as f32),
            Value::I32(v) => Some(*v as f32),
            Value::U32(v) => Some(*v as f32),
            Value::I16(v) => Some(*v as f32),
            Value::U16(v) => Some(*v as f32),
            Value::I8(v) => Some(*v as f32),
            Value::U8(v) => Some(*v as f32),
        }
    }
    
    fn get_u16(&self, name: &str) -> Option<u16> {
        match self.data.get(name)? {
            Value::U16(v) => Some(*v),
            Value::U8(v) => Some(*v as u16),
            Value::U32(v) => Some(*v as u16),
            Value::U64(v) => Some(*v as u16),
            _ => None,
        }
    }
    
    fn get_u8(&self, name: &str) -> Option<u8> {
        match self.data.get(name)? {
            Value::U8(v) => Some(*v),
            Value::U16(v) => Some(*v as u8),
            _ => None,
        }
    }
    
    fn get_i32(&self, name: &str) -> Option<i32> {
        match self.data.get(name)? {
            Value::I32(v) => Some(*v),
            Value::I16(v) => Some(*v as i32),
            Value::I8(v) => Some(*v as i32),
            Value::I64(v) => Some(*v as i32),
            _ => None,
        }
    }
}
