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

#if (CFG_TUH_ENABLED && CFG_TUH_VENDOR)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "host/usbh.h"
#include "host/usbh_classdriver.h"

#include "vendor_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+


typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  uint16_t epin_size;
  uint16_t epout_size;

  uint8_t epin_buf[CFG_TUH_VENDOR_EPIN_BUFSIZE];
  uint8_t epout_buf[CFG_TUH_VENDOR_EPOUT_BUFSIZE];

  uint8_t itf_protocol;
} vendorh_interface_t;


typedef struct
{
  uint8_t inst_count;
  vendorh_interface_t instances[CFG_TUH_VENDOR];
} vendorh_device_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+

static vendorh_device_t _vendorh_dev[CFG_TUH_DEVICE_MAX];

TU_ATTR_ALWAYS_INLINE static inline vendorh_device_t* get_dev(uint8_t dev_addr);
TU_ATTR_ALWAYS_INLINE static inline vendorh_interface_t* get_instance(uint8_t dev_addr, uint8_t instance);
static uint8_t get_instance_id_by_itfnum(uint8_t dev_addr, uint8_t itf);
static uint8_t get_instance_id_by_epaddr(uint8_t dev_addr, uint8_t ep_addr);

//--------------------------------------------------------------------+
// USBH-CLASS API
//--------------------------------------------------------------------+


static void config_driver_mount_complete(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len);

void vendorh_init(void)
{
  tu_memclr(_vendorh_dev, sizeof(_vendorh_dev));
}

bool vendorh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{

  int nbEndpoint = 0;
  TU_LOG1("VENDOR checking Interface Class %u SubClass %u InterfaceProtocol %u\r\n", desc_itf->bInterfaceClass, desc_itf->bInterfaceSubClass, desc_itf->bInterfaceProtocol);
  //ONLY SUPPORT xbox 360 gamepad interface as defined in https://github.com/torvalds/linux/blob/master/drivers/input/joystick/xpad.c
  TU_VERIFY(desc_itf->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC);
  TU_VERIFY(desc_itf->bInterfaceSubClass == 93);
  TU_VERIFY((desc_itf->bInterfaceProtocol == 1) || (desc_itf->bInterfaceProtocol == 129));

  TU_LOG1("VENDOR opening Interface %u (addr = %u) (endPoints %u)\r\n", desc_itf->bInterfaceNumber, dev_addr, desc_itf->bNumEndpoints);

  uint16_t const drv_len = sizeof(tusb_desc_interface_t) +  desc_itf->bNumEndpoints*sizeof(tusb_desc_endpoint_t);
  TU_ASSERT(max_len >= drv_len);
  TU_ASSERT(desc_itf->bNumEndpoints == 2);
  nbEndpoint = desc_itf->bNumEndpoints;

  uint8_t const *p_desc = (uint8_t const *) desc_itf;

  vendorh_device_t* vendor_dev = get_dev(dev_addr);
  TU_ASSERT(vendor_dev->inst_count < CFG_TUH_VENDOR, 0);

  vendorh_interface_t* vendor_itf = get_instance(dev_addr, vendor_dev->inst_count);
  vendor_dev->inst_count++;
  vendor_itf->itf_num   = desc_itf->bInterfaceNumber;
  vendor_itf->itf_protocol = desc_itf->bInterfaceProtocol;

  while (nbEndpoint != 0) {
    p_desc = tu_desc_next(p_desc);
    tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;
    TU_LOG1("VENDOR Endpoint %u type %u input %u size %u xfer %u sync %u usage %u interval %d\r\n", nbEndpoint, desc_ep->bDescriptorType, tu_edpt_dir(desc_ep->bEndpointAddress),
    desc_ep->wMaxPacketSize, desc_ep->bmAttributes.xfer, desc_ep->bmAttributes.sync,  desc_ep->bmAttributes.usage, desc_ep->bInterval);
    if(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType) {
      nbEndpoint--;
        TU_ASSERT( tuh_edpt_open(dev_addr, desc_ep) );

        if(tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_OUT) {
          vendor_itf->ep_out     = desc_ep->bEndpointAddress;
          vendor_itf->epout_size = desc_ep->wMaxPacketSize;
        }
        else {
          vendor_itf->ep_in     = desc_ep->bEndpointAddress;
          vendor_itf->epin_size = desc_ep->wMaxPacketSize;
        }
    }
  }
return true;
}


bool vendorh_set_config(uint8_t dev_addr, uint8_t itf_num) {

  uint8_t const instance    = get_instance_id_by_itfnum(dev_addr, itf_num);
  vendorh_interface_t* vendor_itf = get_instance(dev_addr, instance);

  TU_LOG1("VENDOR SET CONFIG %u  %u\r\n", dev_addr, itf_num);

  config_driver_mount_complete(dev_addr, instance, NULL, 0);

  return true;
}

bool tuh_vendor_send_packet_out(uint8_t dev_addr, uint8_t instance, uint8_t *buffer, uint8_t size) {
  vendorh_interface_t* vendor_itf = get_instance(dev_addr, instance);
  TU_VERIFY( usbh_edpt_claim(dev_addr, vendor_itf->ep_out) );
  TU_LOG1("VENDOR SEND PACKET%u %d(%d)\r\n", dev_addr, size, vendor_itf->epout_size);
  usbh_edpt_xfer(dev_addr, vendor_itf->ep_out, buffer, size);
}


bool tuh_vendor_protocol_get(uint8_t dev_addr , uint8_t instance, uint8_t *val) {
  vendorh_interface_t* vendor_itf = get_instance(dev_addr, instance);
  *val = vendor_itf->itf_protocol;
  return true;
}


bool tuh_vendor_receive_report(uint8_t dev_addr, uint8_t instance)
{
  vendorh_interface_t* vendor_itf = get_instance(dev_addr, instance);
  TU_LOG1("VENDOR RECEIVE REPORT %u\r\n", dev_addr);
  // claim endpoint
  TU_VERIFY( usbh_edpt_claim(dev_addr, vendor_itf->ep_in) );
  TU_LOG1("VENDOR VERIFIED REPORT %u %d\r\n", dev_addr, vendor_itf->epin_size);
  return usbh_edpt_xfer(dev_addr, vendor_itf->ep_in, vendor_itf->epin_buf, vendor_itf->epin_size);
}


static void config_driver_mount_complete(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  vendorh_interface_t* vendor_itf = get_instance(dev_addr, instance);
  TU_LOG1("VENDOR MOUNT COMPLETE %u\r\n", dev_addr);
  // enumeration is complete
  tuh_vendor_mount_cb(dev_addr, instance, desc_report, desc_len);

  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(dev_addr, vendor_itf->itf_num);
}

bool vendorh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) result;

  uint8_t const dir = tu_edpt_dir(ep_addr);
  uint8_t const instance = get_instance_id_by_epaddr(dev_addr, ep_addr);
  vendorh_interface_t* vendor_itf = get_instance(dev_addr, instance);
TU_LOG1("VENDOR vendorh_xfer_cb (%u, %u)\r\n", dev_addr, instance);
  if ( dir == TUSB_DIR_IN )
  {
    TU_LOG1("Get Report callback (%u, %u)\r\n", dev_addr, instance);
    tuh_vendor_report_received_cb(dev_addr, instance, vendor_itf->epin_buf, xferred_bytes);
  }else
  {
    TU_LOG1("Get Report sent callback (%u, %u)\r\n", dev_addr, instance);
    if (tuh_vendor_report_sent_cb) tuh_vendor_report_sent_cb(dev_addr, instance, vendor_itf->epout_buf, xferred_bytes);
  }

  return true;
}

void vendorh_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );

  vendorh_device_t* vendor_dev = get_dev(dev_addr);

  if (tuh_vendor_umount_cb)
  {
    for (uint8_t inst = 0; inst < vendor_dev->inst_count; inst++ ) tuh_vendor_umount_cb(dev_addr, inst);
  }

  tu_memclr(vendor_dev, sizeof(vendorh_device_t));
}

//--------------------------------------------------------------------+
// Helper
//--------------------------------------------------------------------+

// Get Device by address
TU_ATTR_ALWAYS_INLINE static inline vendorh_device_t* get_dev(uint8_t dev_addr)
{
  return &_vendorh_dev[dev_addr-1];
}

// Get Interface by instance number
TU_ATTR_ALWAYS_INLINE static inline vendorh_interface_t* get_instance(uint8_t dev_addr, uint8_t instance)
{
  return &_vendorh_dev[dev_addr-1].instances[instance];
}

// Get instance ID by interface number
static uint8_t get_instance_id_by_itfnum(uint8_t dev_addr, uint8_t itf)
{
  for ( uint8_t inst = 0; inst < CFG_TUH_HID; inst++ )
  {
    vendorh_interface_t *vendor = get_instance(dev_addr, inst);

    if ( (vendor->itf_num == itf) && (vendor->ep_in || vendor->ep_out) ) return inst;
  }

  return 0xff;
}

// Get instance ID by endpoint address
static uint8_t get_instance_id_by_epaddr(uint8_t dev_addr, uint8_t ep_addr)
{
  for ( uint8_t inst = 0; inst < CFG_TUH_HID; inst++ )
  {
    vendorh_interface_t *vendor = get_instance(dev_addr, inst);

    if ( (ep_addr == vendor->ep_in) || ( ep_addr == vendor->ep_out) ) return inst;
  }

  return 0xff;
}

#endif
