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

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUH_MSC

#include "host/usbh.h"
#include "host/usbh_classdriver.h"

#include "msc_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
enum
{
  MSC_STAGE_IDLE = 0,
  MSC_STAGE_CMD,
  MSC_STAGE_DATA,
  MSC_STAGE_STATUS,
};

typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  uint8_t max_lun;

  volatile bool configured; // Receive SET_CONFIGURE
  volatile bool mounted;    // Enumeration is complete

  struct {
    uint32_t block_size;
    uint32_t block_count;
  } capacity[CFG_TUH_MSC_MAXLUN];

  //------------- SCSI -------------//
  uint8_t stage;
  void*   buffer;
  tuh_msc_complete_cb_t complete_cb;

  msc_cbw_t cbw;
  msc_csw_t csw;
}msch_interface_t;

CFG_TUSB_MEM_SECTION static msch_interface_t _msch_itf[CFG_TUH_DEVICE_MAX];

// buffer used to read scsi information when mounted
// largest response data currently is inquiry TODO Inquiry is not part of enum anymore
CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(4)
static uint8_t _msch_buffer[sizeof(scsi_inquiry_resp_t)];

TU_ATTR_ALWAYS_INLINE
static inline msch_interface_t* get_itf(uint8_t dev_addr)
{
  return &_msch_itf[dev_addr-1];
}

//--------------------------------------------------------------------+
// PUBLIC API
//--------------------------------------------------------------------+
uint8_t tuh_msc_get_maxlun(uint8_t dev_addr)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->max_lun;
}

uint32_t tuh_msc_get_block_count(uint8_t dev_addr, uint8_t lun)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->capacity[lun].block_count;
}

uint32_t tuh_msc_get_block_size(uint8_t dev_addr, uint8_t lun)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->capacity[lun].block_size;
}

bool tuh_msc_mounted(uint8_t dev_addr)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->mounted;
}

bool tuh_msc_ready(uint8_t dev_addr)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->mounted && !usbh_edpt_busy(dev_addr, p_msc->ep_in);
}

//--------------------------------------------------------------------+
// PUBLIC API: SCSI COMMAND
//--------------------------------------------------------------------+
static inline void cbw_init(msc_cbw_t *cbw, uint8_t lun)
{
  tu_memclr(cbw, sizeof(msc_cbw_t));
  cbw->signature = MSC_CBW_SIGNATURE;
  cbw->tag       = 0x54555342; // TUSB
  cbw->lun       = lun;
}

bool tuh_msc_scsi_command(uint8_t dev_addr, msc_cbw_t const* cbw, void* data, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  // TODO claim endpoint

  TU_LOG1("Scsi Cmd: ");
  for (int i =0 ; i<cbw->cmd_len; i++) TU_LOG1("0x%x ", cbw->command[i]);
  TU_LOG1("\n");

  p_msc->cbw = *cbw;
  p_msc->stage = MSC_STAGE_CMD;
  p_msc->buffer = data;
  p_msc->complete_cb = complete_cb;

  TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_out, (uint8_t*) &p_msc->cbw, sizeof(msc_cbw_t)));

  return true;
}

bool tuh_msc_read_capacity(uint8_t dev_addr, uint8_t lun, scsi_read_capacity10_resp_t* response, tuh_msc_complete_cb_t complete_cb)
{
   msch_interface_t* p_msc = get_itf(dev_addr);
   TU_VERIFY(p_msc->configured);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = sizeof(scsi_read_capacity10_resp_t);
  cbw.dir        = TUSB_DIR_IN_MASK;
  cbw.cmd_len    = sizeof(scsi_read_capacity10_t);
  cbw.command[0] = SCSI_CMD_READ_CAPACITY_10;

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb);
}

bool tuh_msc_inquiry(uint8_t dev_addr, uint8_t lun, scsi_inquiry_resp_t* response, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = sizeof(scsi_inquiry_resp_t);
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_inquiry_t);

  scsi_inquiry_t const cmd_inquiry =
  {
    .cmd_code     = SCSI_CMD_INQUIRY,
    .alloc_length = sizeof(scsi_inquiry_resp_t)
  };
  memcpy(cbw.command, &cmd_inquiry, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb);
}

bool tuh_msc_test_unit_ready(uint8_t dev_addr, uint8_t lun, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir         = TUSB_DIR_OUT;
  cbw.cmd_len     = sizeof(scsi_test_unit_ready_t);
  cbw.command[0]  = SCSI_CMD_TEST_UNIT_READY;
  cbw.command[1]  = lun; // according to wiki TODO need verification

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb);
}

bool tuh_msc_request_sense(uint8_t dev_addr, uint8_t lun, void *resposne, tuh_msc_complete_cb_t complete_cb)
{
  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 18; // TODO sense response
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_request_sense_t);

  scsi_request_sense_t const cmd_request_sense =
  {
    .cmd_code     = SCSI_CMD_REQUEST_SENSE,
    .alloc_length = 18
  };

  memcpy(cbw.command, &cmd_request_sense, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, resposne, complete_cb);
}

bool tuh_msc_read10(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count*p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read10_t);

  scsi_read10_t const cmd_read10 =
  {
    .cmd_code    = SCSI_CMD_READ_10,
    .lba         = tu_htonl(lba),
    .block_count = tu_htons(block_count)
  };

  memcpy(cbw.command, &cmd_read10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool  tuh_msc_read_cd(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint32_t block_count, bool subQ, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  uint32_t block_size = p_msc->capacity[lun].block_size;

  block_size = 2352;
  if (subQ) block_size += 96;

  cbw.total_bytes = block_count*block_size; // arevoir
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_cd_t);

  scsi_read_cd_t const cmd_read_cd =
  {
    .cmd_code    = SCSI_CMD_READ_CD,
    .lba         = tu_htonl(lba),
    .block_count[0] = (block_count>>16)&0xFF, //MSB
    .block_count[1] = (block_count>>8)&0xFF,
    .block_count[2] = block_count&0xFF, //LSB
    .sector_type = 0b000,
    .sync = 1,
    .header_code = 0b11,
    .user_data = 1,
    .edc_ecc = 1,
    .sub_channel = subQ?0b001:0b000,
  };

  memcpy(cbw.command, &cmd_read_cd, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}


bool tuh_msc_read_sub_channel(uint8_t dev_addr, uint8_t lun, void * buffer, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 12+4; // arevoir
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_sub_channel_t);

  scsi_read_sub_channel_t const cmd_read_sub_channel=
  {
    .cmd_code    = SCSI_CMD_READ_SUB_CHANNEL,
    .subq        = 1,
    .msf         = 1,
    .parameter   = 0x01,
    .alloc_length = tu_htons(12+4),
  };

  memcpy(cbw.command, &cmd_read_sub_channel, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_read_header(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 8;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_header_t);

  scsi_read_header_t const cmd_read_header=
  {
    .cmd_code    = SCSI_CMD_READ_HEADER,
    .msf         = 1,
    .lba         = tu_htonl(lba),
    .alloc_length = tu_htons(8),
  };

  memcpy(cbw.command, &cmd_read_header, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_start_stop(uint8_t dev_addr, uint8_t lun, bool start, bool load_eject, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_start_stop_unit_t);

  scsi_start_stop_unit_t const cmd_start_stop =
  {
    .cmd_code        = SCSI_CMD_START_STOP_UNIT,
    .start           = start,
    .load_eject      = load_eject,
    .lun = lun,
  };

  memcpy(cbw.command, &cmd_start_stop, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb);
}

bool tuh_msc_write10(uint8_t dev_addr, uint8_t lun, void const * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count*p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_OUT;
  cbw.cmd_len     = sizeof(scsi_write10_t);

  scsi_write10_t const cmd_write10 =
  {
    .cmd_code    = SCSI_CMD_WRITE_10,
    .lba         = tu_htonl(lba),
    .block_count = tu_htons(block_count)
  };

  memcpy(cbw.command, &cmd_write10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, (void*)(uintptr_t) buffer, complete_cb);
}

bool tuh_msc_read_toc(uint8_t dev_addr, uint8_t lun, void * buffer, uint8_t msf, uint8_t starting_track, uint8_t nb_tracks, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 4 + 8*nb_tracks;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_toc_t);

  scsi_read_toc_t const cmd_read_toc =
  {
    .cmd_code    = SCSI_CMD_READ_TOC,
    .msf         = msf,
    .starting_track = starting_track,
    .alloc_length = tu_htons(4 + 8*nb_tracks), //MAXIMUM TOC length
  };

  memcpy(cbw.command, &cmd_read_toc, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_set_speed(uint8_t dev_addr, uint8_t lun, uint16_t read_speed, uint16_t write_speed, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_set_speed_t);

  scsi_set_speed_t const cmd_set_speed =
  {
    .cmd_code    = SCSI_CMD_SET_SPEED,
    .read_speed  = tu_htons(read_speed),
    .write_speed = tu_htons(write_speed),
  };

  memcpy(cbw.command, &cmd_set_speed, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb);
}

#if 0
// MSC interface Reset (not used now)
bool tuh_msc_reset(uint8_t dev_addr)
{
  tusb_control_request_t const new_request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = MSC_REQ_RESET,
    .wValue   = 0,
    .wIndex   = p_msc->itf_num,
    .wLength  = 0
  };
  TU_ASSERT( usbh_control_xfer( dev_addr, &new_request, NULL ) );
}
#endif

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+
void msch_init(void)
{
  tu_memclr(_msch_itf, sizeof(_msch_itf));
}

void msch_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );

  msch_interface_t* p_msc = get_itf(dev_addr);

  // invoke Application Callback
  if (p_msc->mounted && tuh_msc_umount_cb) tuh_msc_umount_cb(dev_addr);

  tu_memclr(p_msc, sizeof(msch_interface_t));
}

bool msch_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  msc_cbw_t const * cbw = &p_msc->cbw;
  msc_csw_t       * csw = &p_msc->csw;
TU_LOG2("MSC Xfer stage %d %x %x\n", p_msc->stage, cbw->total_bytes, p_msc->buffer);
  switch (p_msc->stage)
  {
    case MSC_STAGE_CMD:
      // Must be Command Block
      TU_ASSERT(ep_addr == p_msc->ep_out &&  event == XFER_RESULT_SUCCESS && xferred_bytes == sizeof(msc_cbw_t));

      if ( cbw->total_bytes && p_msc->buffer )
      {
        // Data stage if any
        p_msc->stage = MSC_STAGE_DATA;

        uint8_t const ep_data = (cbw->dir & TUSB_DIR_IN_MASK) ? p_msc->ep_in : p_msc->ep_out;
        TU_ASSERT(usbh_edpt_xfer(dev_addr, ep_data, p_msc->buffer, cbw->total_bytes));
      }else
      {
        // Status stage
        p_msc->stage = MSC_STAGE_STATUS;
        TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_in, (uint8_t*) &p_msc->csw, sizeof(msc_csw_t)));
      }
    break;

    case MSC_STAGE_DATA:
      // Status stage
      p_msc->stage = MSC_STAGE_STATUS;
      TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_in, (uint8_t*) &p_msc->csw, sizeof(msc_csw_t)));
    break;

    case MSC_STAGE_STATUS:
      // SCSI op is complete
      p_msc->stage = MSC_STAGE_IDLE;

      if (p_msc->complete_cb) p_msc->complete_cb(dev_addr, cbw, csw);
    break;

    // unknown state
    default: break;
  }

  return true;
}

//--------------------------------------------------------------------+
// MSC Enumeration
//--------------------------------------------------------------------+

static bool config_get_maxlun_complete (uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result);
static bool config_test_unit_ready_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);
static bool config_request_sense_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);
static bool config_read_capacity_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);

bool msch_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  TU_VERIFY (MSC_SUBCLASS_SCSI == desc_itf->bInterfaceSubClass &&
             MSC_PROTOCOL_BOT  == desc_itf->bInterfaceProtocol);

  // msc driver length is fixed
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + desc_itf->bNumEndpoints*sizeof(tusb_desc_endpoint_t);
  TU_ASSERT(drv_len <= max_len);

  msch_interface_t* p_msc = get_itf(dev_addr);
  tusb_desc_endpoint_t const * ep_desc = (tusb_desc_endpoint_t const *) tu_desc_next(desc_itf);

  for(uint32_t i=0; i<2; i++)
  {
    TU_ASSERT(TUSB_DESC_ENDPOINT == ep_desc->bDescriptorType && TUSB_XFER_BULK == ep_desc->bmAttributes.xfer);
    TU_ASSERT(usbh_edpt_open(rhport, dev_addr, ep_desc));

    if ( tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN )
    {
      p_msc->ep_in = ep_desc->bEndpointAddress;
    }else
    {
      p_msc->ep_out = ep_desc->bEndpointAddress;
    }

    ep_desc = (tusb_desc_endpoint_t const *) tu_desc_next(ep_desc);
  }

  p_msc->itf_num = desc_itf->bInterfaceNumber;

  return true;
}

bool msch_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_ASSERT(p_msc->itf_num == itf_num);

  p_msc->configured = true;

  //------------- Get Max Lun -------------//
  TU_LOG1("MSC Get Max Lun\r\n");
  tusb_control_request_t request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = MSC_REQ_GET_MAX_LUN,
    .wValue   = 0,
    .wIndex   = itf_num,
    .wLength  = 1
  };
  TU_ASSERT(tuh_control_xfer(dev_addr, &request, &p_msc->max_lun, config_get_maxlun_complete));

  return true;
}

static bool config_get_maxlun_complete (uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result)
{
  (void) request;

  msch_interface_t* p_msc = get_itf(dev_addr);

  // STALL means zero
  p_msc->max_lun = (XFER_RESULT_SUCCESS == result) ? _msch_buffer[0] : 0;
  p_msc->max_lun++; // MAX LUN is minus 1 by specs


  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(dev_addr, p_msc->itf_num);

  // // TODO multiple LUN support
  if (tuh_msc_enumerated_cb) tuh_msc_enumerated_cb(dev_addr);
  else {
    uint8_t const lun = 0;
    checkForMedia(dev_addr, lun);
  }

  return true;
}

void checkForMedia(uint8_t dev_addr, uint8_t lun) {
  TU_LOG1("checkForMedia\r\n");
  tuh_msc_test_unit_ready(dev_addr, lun, config_test_unit_ready_complete);
}

static bool config_test_unit_ready_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
  if (csw->status == 0)
  {
    // Unit is ready, read its capacity
    TU_LOG1("SCSI Read Capacity\r\n");
    if (tuh_msc_ready_cb) tuh_msc_ready_cb(dev_addr, true);
    tuh_msc_read_capacity(dev_addr, cbw->lun, (scsi_read_capacity10_resp_t*) ((void*) _msch_buffer), config_read_capacity_complete);
  }else
  {
    // Note: During enumeration, some device fails Test Unit Ready and require a few retries
    // with Request Sense to start working !!
    // TODO limit number of retries
    if (tuh_msc_ready_cb) tuh_msc_ready_cb(dev_addr, false);
    TU_LOG1("SCSI Request Sense %x\r\n", csw->status);
    TU_ASSERT(tuh_msc_request_sense(dev_addr, cbw->lun, _msch_buffer, config_request_sense_complete));
  }

  return true;
}

static bool config_request_sense_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
  // printf("Sense key %x\n", _msch_buffer[2] & 0xF);

  TU_ASSERT(csw->status == 0);
  if (!tuh_msc_enumerated_cb) TU_ASSERT(tuh_msc_test_unit_ready(dev_addr, cbw->lun, config_test_unit_ready_complete));
  else tuh_msc_enumerated_cb(dev_addr);
  return true;
}

static bool config_read_capacity_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
  TU_ASSERT(csw->status == 0);

  msch_interface_t* p_msc = get_itf(dev_addr);

  // Capacity response field: Block size and Last LBA are both Big-Endian
  scsi_read_capacity10_resp_t* resp = (scsi_read_capacity10_resp_t*) ((void*) _msch_buffer);
  p_msc->capacity[cbw->lun].block_count = tu_ntohl(resp->last_lba) + 1;
  p_msc->capacity[cbw->lun].block_size = tu_ntohl(resp->block_size);

  // Mark enumeration is complete
  p_msc->mounted = true;
  if (tuh_msc_mount_cb) tuh_msc_mount_cb(dev_addr);

  return true;
}

#endif
