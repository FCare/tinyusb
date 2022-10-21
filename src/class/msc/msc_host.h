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

#ifndef _TUSB_MSC_HOST_H_
#define _TUSB_MSC_HOST_H_

#include "msc.h"

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

#ifndef CFG_TUH_MSC_MAXLUN
#define CFG_TUH_MSC_MAXLUN  4
#endif


typedef bool (*tuh_msc_complete_cb_t)(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Check if device supports MassStorage interface.
// This function true after tuh_msc_mounted_cb() and false after tuh_msc_unmounted_cb()
bool tuh_msc_mounted(uint8_t dev_addr);

// Check if the interface is currently ready or busy transferring data
bool tuh_msc_ready(uint8_t dev_addr);

// Get Max Lun
uint8_t tuh_msc_get_maxlun(uint8_t dev_addr);

// Get number of block
uint32_t tuh_msc_get_block_count(uint8_t dev_addr, uint8_t lun);

// Get block size in bytes
uint32_t tuh_msc_get_block_size(uint8_t dev_addr, uint8_t lun);

// Perform a full SCSI command (cbw, data, csw) in non-blocking manner.
// Complete callback is invoked when SCSI op is complete.
// return true if success, false if there is already pending operation.
bool tuh_msc_scsi_command(uint8_t dev_addr, msc_cbw_t const* cbw, void* data, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Inquiry command
// Complete callback is invoked when SCSI op is complete.
bool tuh_msc_inquiry(uint8_t dev_addr, uint8_t lun, scsi_inquiry_resp_t* response, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Test Unit Ready command
// Complete callback is invoked when SCSI op is complete.
bool tuh_msc_test_unit_ready(uint8_t dev_addr, uint8_t lun, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Request Sense 10 command
// Complete callback is invoked when SCSI op is complete.
bool tuh_msc_request_sense(uint8_t dev_addr, uint8_t lun, void *resposne, tuh_msc_complete_cb_t complete_cb);

bool tuh_msc_mode_sense(uint8_t dev_addr, uint8_t lun, uint8_t page_code, uint8_t dbd,  uint8_t llba, uint16_t alloc_length, uint8_t* response, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Read 10 command. Read n blocks starting from LBA to buffer
// Complete callback is invoked when SCSI op is complete.
bool  tuh_msc_read10(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb);
bool  tuh_msc_read10_async(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb);

bool  tuh_msc_read_cd(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint32_t block_count, bool subQ, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Write 10 command. Write n blocks starting from LBA to device
// Complete callback is invoked when SCSI op is complete.
bool tuh_msc_write10(uint8_t dev_addr, uint8_t lun, void const * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb);
bool tuh_msc_write10_sync(uint8_t dev_addr, uint8_t lun, void const * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Read Capacity 10 command
// Complete callback is invoked when SCSI op is complete.
// Note: during enumeration, host stack already carried out this request. Application can retrieve capacity by
// simply call tuh_msc_get_block_count() and tuh_msc_get_block_size()
bool tuh_msc_read_capacity(uint8_t dev_addr, uint8_t lun, scsi_read_capacity10_resp_t* response, tuh_msc_complete_cb_t complete_cb);

// Perform SCSI Read Toc command.
// Complete callback is invoked when SCSI op is complete.
bool tuh_msc_read_toc(uint8_t dev_addr, uint8_t lun, void * buffer, uint8_t msf, uint8_t starting_track, uint8_t nb_tracks, tuh_msc_complete_cb_t complete_cb);

bool tuh_msc_start_stop(uint8_t dev_addr, uint8_t lun, bool start, bool load_eject, tuh_msc_complete_cb_t complete_cb);

bool tuh_msc_read_sub_channel(uint8_t dev_addr, uint8_t lun, void * buffer, tuh_msc_complete_cb_t complete_cb);

bool tuh_msc_read_header(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, tuh_msc_complete_cb_t complete_cb);

bool tuh_msc_set_speed(uint8_t dev_addr, uint8_t lun, uint16_t read_speed, uint16_t write_speed, tuh_msc_complete_cb_t complete_cb);

bool checkForMedia(uint8_t dev_addr, uint8_t lun);

bool CheckCDCapabilities(uint8_t dev_addr, bool *canBeLoaded, bool* canBeEjected);

//------------- Application Callback -------------//

//Invoked when drive is READY
TU_ATTR_WEAK void tuh_msc_ready_cb(uint8_t dev_addr, bool ready);

// Invoked when a device with MassStorage interface is mounted
TU_ATTR_WEAK void tuh_msc_mount_cb(uint8_t dev_addr);

// Invoked when a device with MassStorage interface is unmounted
TU_ATTR_WEAK void tuh_msc_umount_cb(uint8_t dev_addr);

TU_ATTR_WEAK void tuh_msc_enumerated_cb(uint8_t dev_addr);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+

void msch_init       (void);
bool msch_open       (uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len);
bool msch_set_config (uint8_t dev_addr, uint8_t itf_num);
void msch_close      (uint8_t dev_addr);
bool msch_xfer_cb    (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_MSC_HOST_H_ */
