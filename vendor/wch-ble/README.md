# WCH BLE libraries

WCH's BLE controller and host (TMOS) as static libraries, with their C
headers, for the `ble` feature. Unmodified copies from WCH's EVT packages
(Apache-2.0):

| File | Chips | Version | Source |
|------|-------|---------|--------|
| `libCH58xBLE.a`, `CH58xBLE_LIB.h` | CH584, CH585 | `CH585_BLE_LIB_V1.4` | [openwch/ch585](https://github.com/openwch/ch585) `EVT/EXAM/BLE/LIB`, commit db88919 |
| `libCH59xBLE.a`, `CH59xBLE_LIB.h` | CH591, CH592 | `CH59x_BLE_LIB_V1.4` | [openwch/ch592](https://github.com/openwch/ch592) `EVT/EXAM/BLE/LIB`, commit a46e008 (renamed from `LIBCH59xBLE.a`) |

The headers are GBK-encoded, with Chinese comments, as WCH ships them. The
bindings in `src/ble` are written from them.
