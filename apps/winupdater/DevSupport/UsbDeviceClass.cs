/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
using System;

namespace DevSupport.DeviceManager
{
    public sealed class UsbDeviceClass : DeviceClass
    {
        /// <summary>
        /// Initializes a new instance of the UsbDeviceClass class.
        /// </summary>
        public UsbDeviceClass(UsbDeviceClass.Scope scope)
            : base(Guid.Empty /*Win32.GUID_DEVINTERFACE_USB_DEVICE*/, Win32.GUID_DEVCLASS_USB, "USB")
        {
            _Scope = scope;
        }

        internal override Device CreateDevice(Win32.SP_DEVINFO_DATA deviceInfoData, String path)
        {
            return new UsbDevice(deviceInfoData.devInst, path);
        }

        public UsbDevice FindDeviceByPath(String devicePath)
        {

            if (String.IsNullOrEmpty(devicePath))
	        {
		        return null;
	        }

	        // Find the Hub in our list of hubs
	        foreach ( Device dev in Devices )
	        {
                if (dev != null)
                {
                    if (String.Compare(dev.Path, devicePath, true) == 0)
                    {
                        return dev;
                    }
                }
	        }

	        return null;
        }

        public UsbHub FindHubByDriver(String driverName)
        {
            if (String.IsNullOrEmpty(driverName))
	        {
		        return null;
	        }

	        // Find the Hub in our list of hubs
	        foreach ( UsbHub hub in Devices )
	        {
                if (hub != null)
                {
                    if (String.Compare(hub.Driver, driverName, true) == 0)
                    {
                        return hub;
                    }
                }
	        }

	        return null;
        }
    }
}
/*
void usb::HubMgr::RefreshHubs()
{
	// Init the list of hubs
	DeviceClass::Devices();

	std::list<Device*>::iterator deviceItem;

	for ( deviceItem = _devices.begin(); deviceItem != _devices.end(); ++deviceItem )
	{
		usb::Hub* pHub = dynamic_cast<usb::Hub*>(*deviceItem);
		pHub->RefreshPort(0); // 0 means refresh all the ports
	}
}

*/