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
using System.ComponentModel;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;

using Microsoft.Win32.SafeHandles;

namespace DevSupport.DeviceManager
{
    /// <summary>
    /// A Human Interface Device (HID).
    /// </summary>
    public class HidDevice : Device
    {
        internal HidDevice(UInt32 deviceInstance, string path)
            : base(deviceInstance, path)
        {
            AllocateIoBuffers();
        }

        //------------------------------------------------------------------------------
        // HID Device Variables
        //------------------------------------------------------------------------------
        [TypeConverter(typeof(ExpandableObjectConverter))]
        public NativeMethods.HIDP_CAPS HidCapabilities
        {
            get 
            {
                return _HidCapabilities;
            }
        }
        private NativeMethods.HIDP_CAPS _HidCapabilities;

//        private HidDataReport _ReadReport;
//        private HidDataReport _WriteReport;

        //------------------------------------------------------------------------------
        // HID Device Functions
        //------------------------------------------------------------------------------
        // Modiifes _Capabilities member variable
        private int AllocateIoBuffers()
        {
            int error = NativeMethods.ERROR_SUCCESS;

            // Open the device
            NativeMethods.SECURITY_ATTRIBUTES secAttribs = new NativeMethods.SECURITY_ATTRIBUTES();
            secAttribs.nLength = Marshal.SizeOf(typeof(NativeMethods.SECURITY_ATTRIBUTES));
            SafeFileHandle hHidDevice = NativeMethods.CreateFile(Path, 0, 0, ref secAttribs, NativeMethods.OPEN_EXISTING, 0, IntPtr.Zero);

            if( hHidDevice.IsInvalid )
            {
                error = Marshal.GetLastWin32Error();
//                Trace.WriteLine(String.Format(" HidDevice.AllocateIoBuffers() ERROR:{0}"), error);
                return error;
            }

            // Get the Capabilities including the max size of the report buffers
            IntPtr PreparsedData;
            if ( !NativeMethods.HidD_GetPreparsedData(hHidDevice, out PreparsedData) )
            {
                hHidDevice.Close();
                error = NativeMethods.ERROR_GEN_FAILURE;
//                Trace.WriteLine(String.Format(" HidDevice.AllocateIoBuffers() ERROR:{0}"), error);
                return error;
            }

            if (NativeMethods.HidP_GetCaps(PreparsedData, ref _HidCapabilities) != NativeMethods.HIDP_STATUS_SUCCESS)
            {
                hHidDevice.Close();
                NativeMethods.HidD_FreePreparsedData(PreparsedData);
                error = NativeMethods.HIDP_STATUS_INVALID_PREPARSED_DATA;
//                Trace.WriteLine(String.Format(" HidDevice.AllocateIoBuffers() ERROR:{0}"), error);
                return error;
            }

            hHidDevice.Close();
            NativeMethods.HidD_FreePreparsedData(PreparsedData);
            return NativeMethods.ERROR_SUCCESS;
        }

    } // class HidDevice
}
