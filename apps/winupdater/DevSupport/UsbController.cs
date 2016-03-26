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
using System.Runtime.InteropServices;

using Microsoft.Win32.SafeHandles;

namespace DevSupport.DeviceManager
{
    /// <summary>
    /// A USB Controller device.
    /// </summary>
    public sealed class UsbController : Device
    {
        /// <summary>
        /// Creates an instance of a UsbController. Do not call this constructor
        /// directly. Instead, use the UsbController.Instance to call the
        /// CreateDevice method.
        /// </summary>
        internal UsbController(UInt32 deviceInstance, string path)
            : base(deviceInstance, path)
        {
        }

        /// <summary>
        /// The child device of a USB Host Controller is a USB Root UsbHub.
        /// </summary>
        public override Device Child
        {
            get
            {
                UInt32 childDevInst = 0;
                if (NativeMethods.CM_Get_Child(out childDevInst, DeviceInstance, 0) == NativeMethods.CONFIGRET.CR_SUCCESS)
                {
                    foreach (UsbHub hub in UsbHubClass.Instance)
                    {
                        if (hub.DeviceInstance == childDevInst)
                            return hub;
                    }
                }

                return null;
            }
        }

        /// <summary>
        /// The USB Root UsbHub of the USB Host Controller.
        /// </summary>
        public UsbHub RootHub
        {
            get
            {
                if (_RootHub == null)
                {
                    _RootHub = Child as UsbHub;
                }
                
                return _RootHub;
            }
        }
        private UsbHub _RootHub;

        /// <summary>
        /// Get the USB Root Hub device name by querying the device. Don't really need to use this
        /// property since we know the RootHub is the Child device of the Controller. We can get the
        /// same info through the registry without having to open/close the hardware.
        /// </summary>
        public String RootHubFileName
        {
            get 
            {
                if (String.IsNullOrEmpty(_RootHubFileName))
                {
                    _RootHubFileName = this.GetRootHubFileName();
                }
                
                return _RootHubFileName;
            }
        }
        private String _RootHubFileName;

        /// <summary>
        /// Worker function for the RootHubFileName filename property.
        /// </summary>
        private String GetRootHubFileName()
        {
            String fileName = String.Empty;

            SafeFileHandle hController = Open();
            if ( hController.IsInvalid )
            {
                int error = Marshal.GetLastWin32Error();
                throw new Win32Exception(error);
            }
            
            int nBytesReturned;
            NativeMethods.USB_ROOT_HUB_NAME hubName = new NativeMethods.USB_ROOT_HUB_NAME();
            int nBytes = Marshal.SizeOf(hubName);
            IntPtr ptrHubName = Marshal.AllocHGlobal(nBytes);

            // get the UsbHub Name
            if ( NativeMethods.DeviceIoControl(hController, NativeMethods.IOCTL_USB_GET_ROOT_HUB_NAME, ptrHubName,
                nBytes, ptrHubName, nBytes, out nBytesReturned, IntPtr.Zero) )
            {
                hubName = (NativeMethods.USB_ROOT_HUB_NAME)Marshal.PtrToStructure(ptrHubName, typeof(NativeMethods.USB_ROOT_HUB_NAME));
                fileName = @"\\.\" + hubName.RootHubName;

                hController.Close();
                Marshal.FreeHGlobal(ptrHubName);
            }
            else
            {
                int error = Marshal.GetLastWin32Error();
                hController.Close();
                Marshal.FreeHGlobal(ptrHubName);

                throw new Win32Exception(error);
            }

            return fileName;
        }

        /// <summary>
        /// Opens the USB Controller device. Returns a handle to use with DeviceIoControl() commands.
        /// </summary>
        private SafeFileHandle Open()
        {
            // Needed for Win2000
            NativeMethods.SECURITY_ATTRIBUTES securityAttrib = new NativeMethods.SECURITY_ATTRIBUTES();
            securityAttrib.nLength = Marshal.SizeOf(typeof(NativeMethods.SECURITY_ATTRIBUTES));
//            securityAttrib.bInheritHandle = false;
//            securityAttrib.lpSecurityDescriptor = new IntPtr();
//            securityAttrib.nLength = Marshal.SizeOf(typeof(NativeMethods.SECURITY_ATTRIBUTES));// sizeof(NativeMethods.SECURITY_ATTRIBUTES);
//            IntPtr ptrSecurityAttrib = Marshal.AllocHGlobal(securityAttrib.nLength);
//            Marshal.StructureToPtr(securityAttrib, ptrSecurityAttrib, true);

            return NativeMethods.CreateFile(
                Path.Replace(@"\??", @"\\."),
                NativeMethods.GENERIC_WRITE,
                NativeMethods.FILE_SHARE_WRITE,
                ref securityAttrib,
                NativeMethods.OPEN_EXISTING, 0, IntPtr.Zero);
        }

    } // class UsbController

} // namespace DevSupport.DeviceManager
