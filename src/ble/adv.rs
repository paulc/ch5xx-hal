//! Advertising and scan response data builder.
//!
//! ```ignore
//! let adv = AdvData::new()
//!     .flags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
//!     .uuid16_list(&[0xFFE0])
//!     .short_name("dev");
//! let scan_rsp = AdvData::new().name("device");
//! ```

use super::gap::*;

/// Maximum advertising / scan response data length (legacy advertising).
pub const MAX_ADV_LEN: usize = 31;

/// Advertising or scan response data, built from AD structures.
///
/// Panics if the data exceeds 31 bytes (a configuration error).
#[derive(Clone)]
pub struct AdvData {
    buf: [u8; MAX_ADV_LEN],
    len: usize,
}

impl AdvData {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_ADV_LEN],
            len: 0,
        }
    }

    fn push(&mut self, bytes: &[u8]) {
        let end = self.len + bytes.len();
        assert!(end <= MAX_ADV_LEN, "AdvData too long");
        self.buf[self.len..end].copy_from_slice(bytes);
        self.len = end;
    }

    /// Add an AD structure: length, type, data.
    pub fn add(mut self, ad_type: u8, data: &[u8]) -> Self {
        self.push(&[data.len() as u8 + 1, ad_type]);
        self.push(data);
        self
    }

    /// Flags, e.g. `GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED`.
    pub fn flags(self, flags: u8) -> Self {
        self.add(GAP_ADTYPE_FLAGS, &[flags])
    }

    /// Complete local name.
    pub fn name(self, name: &str) -> Self {
        self.add(GAP_ADTYPE_LOCAL_NAME_COMPLETE, name.as_bytes())
    }

    /// Shortened local name (the start of the complete name), e.g. in the
    /// advertising packet when the complete name is in the scan response.
    pub fn short_name(self, name: &str) -> Self {
        self.add(GAP_ADTYPE_LOCAL_NAME_SHORT, name.as_bytes())
    }

    /// Complete list of 16-bit service UUIDs.
    pub fn uuid16_list(self, uuids: &[u16]) -> Self {
        let mut data = [0u8; MAX_ADV_LEN];
        assert!(uuids.len() * 2 <= MAX_ADV_LEN, "AdvData too long");
        for (i, uuid) in uuids.iter().enumerate() {
            data[2 * i..2 * i + 2].copy_from_slice(&uuid.to_le_bytes());
        }
        self.add(GAP_ADTYPE_16BIT_COMPLETE, &data[..uuids.len() * 2])
    }

    /// Preferred connection interval range (1.25 ms units).
    pub fn conn_interval_range(self, min: u16, max: u16) -> Self {
        let [a, b] = min.to_le_bytes();
        let [c, d] = max.to_le_bytes();
        self.add(GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE, &[a, b, c, d])
    }

    /// TX power level (dBm).
    pub fn tx_power(self, dbm: i8) -> Self {
        self.add(GAP_ADTYPE_POWER_LEVEL, &[dbm as u8])
    }

    /// Manufacturer specific data: company ID and payload.
    pub fn manufacturer(self, company_id: u16, data: &[u8]) -> Self {
        let mut payload = [0u8; MAX_ADV_LEN];
        assert!(data.len() + 2 <= MAX_ADV_LEN, "AdvData too long");
        payload[..2].copy_from_slice(&company_id.to_le_bytes());
        payload[2..2 + data.len()].copy_from_slice(data);
        self.add(GAP_ADTYPE_MANUFACTURER_SPECIFIC, &payload[..2 + data.len()])
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl Default for AdvData {
    fn default() -> Self {
        Self::new()
    }
}
