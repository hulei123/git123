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
    public sealed class HidDeviceClass : DeviceClass
    {
        /// <summary>
        /// Initializes a new instance of the HidDeviceClass class.
        /// </summary>
        private HidDeviceClass()
            : base(NativeMethods.GUID_DEVINTERFACE_HID, NativeMethods.GUID_DEVCLASS_HIDCLASS, "HID")
        { }

        /// <summary>
        /// Gets the single HidDeviceClass instance.
        /// </summary>
        public static HidDeviceClass Instance
        {
            get { return Utils.Singleton<HidDeviceClass>.Instance; }
        }

        internal override Device CreateDevice(UInt32 deviceInstance, String path)
        {
            var device = new HidDevice(deviceInstance, path);
            if (device.Match(0x15a2, 0x0073))
                return device;
            else
                return null;
        }
    }
}
/*
size_t HidDeviceClass::AddFilter(uint16_t vid, uint16_t pid)
{
	CStdString filter;
	filter.Format(_T("hid#vid_%04x&pid_%04x"), vid, pid);
	_filters.push_back(filter);
	return _filters.size();
}

size_t HidDeviceClass::AddFilter(LPCTSTR vid, LPCTSTR pid, LPCTSTR instance)
{
	CStdString filter;
	filter.Format(_T("hid#vid_%s&pid_%s#%s"), vid, pid, instance);
	_filters.push_back(filter);
	return _filters.size();
}
*/
