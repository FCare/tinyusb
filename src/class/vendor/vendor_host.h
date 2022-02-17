/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _TUSB_VENDOR_HOST_H_
#define _TUSB_VENDOR_HOST_H_

#include "common/tusb_common.h"

#ifdef __cplusplus
 extern "C" {
#endif


//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

#ifndef CFG_TUH_VENDOR_EPIN_BUFSIZE
#define CFG_TUH_VENDOR_EPIN_BUFSIZE 64
#endif

#ifndef CFG_TUH_VENDOR_EPOUT_BUFSIZE
#define CFG_TUH_VENDOR_EPOUT_BUFSIZE 64
#endif

//--------------------------------------------------------------------+
// USBH-CLASS DRIVER API
//--------------------------------------------------------------------+
static inline bool tusbh_custom_is_mounted(uint8_t dev_addr, uint16_t vendor_id, uint16_t product_id)
{
  (void) vendor_id; // TODO check this later
  (void) product_id;
//  return (tusbh_device_get_mounted_class_flag(dev_addr) & TU_BIT(TUSB_CLASS_MAPPED_INDEX_END-1) ) != 0;
  return false;
}

//--------------------------------------------------------------------+
// Callbacks (Weak is optional)
//--------------------------------------------------------------------+

// Invoked when device with vendor interface is mounted
void tuh_vendor_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report_desc, uint16_t desc_len);

// Invoked when sent report to device successfully via interrupt endpoint
TU_ATTR_WEAK void tuh_vendor_report_sent_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

// Invoked when device with hid interface is un-mounted
TU_ATTR_WEAK void tuh_vendor_umount_cb(uint8_t dev_addr, uint8_t instance);

// Invoked when received report from device via interrupt endpoint
// Note: if there is report ID (composite), it is 1st byte of report
void tuh_vendor_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

// Invoked when sent report to device successfully via interrupt endpoint
TU_ATTR_WEAK void tuh_vendor_report_sent_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

bool tuh_vendor_receive_report(uint8_t dev_addr, uint8_t instance);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void         vendorh_init(void);
bool         vendorh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len);
bool         vendorh_set_config(uint8_t dev_addr, uint8_t itf_num);
bool         vendorh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
void         vendorh_close(uint8_t dev_addr);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_VENDOR_HOST_H_ */
