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
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;

using Microsoft.Win32.SafeHandles;

namespace DevSupport.DeviceManager
{
    public sealed class UsbControllerClass : DeviceClass
    {
        /// <summary>
        /// Initializes a new instance of the UsbControllerClass class.
        /// </summary>
        private UsbControllerClass()
            : base(NativeMethods.GUID_DEVINTERFACE_USB_HOST_CONTROLLER, NativeMethods.GUID_DEVCLASS_USB, "PCI")
        { }

        /// <summary>
        /// Gets the single UsbControllerClass instance.
        /// </summary>
        public static UsbControllerClass Instance
        {
            get { return Utils.Singleton<UsbControllerClass>.Instance; }
        }

        internal override Device CreateDevice(UInt32 deviceInstance, String path)
        {
            return new UsbController(deviceInstance, path);
        }

        /// <summary>
        /// Gets a set of Device Instances described by the Device Class that can be used to
        /// create instances of the Devices. Override allows extension for DeviceClass-derived types.
        /// In the USB Controller case, in Win2K, we add controllers named \\.\HCD0.
        /// </summary>
        /// <returns>Dictionary( DevInst, Path )</returns>
        protected override Dictionary<UInt32, String> GetDevInstDataSet()
        {
            // Call the base class to get the Device Instances using the 
            // GUID_DEVINTERFACE_USB_HOST_CONTROLLER method.
            Dictionary<UInt32, String> devInstDataSet = base.GetDevInstDataSet();

            // Add something else for Win2K 
            if (Environment.OSVersion.Version < new Version(5, 1))
            {
                // Iterate over some Host Controller names and try to open them.
                for (int hcNum = 0; hcNum < 10; ++hcNum)
                {
                    String controllerName = String.Format(@"\\.\HCD{0}", hcNum);

                    NativeMethods.SECURITY_ATTRIBUTES secAttribs = new NativeMethods.SECURITY_ATTRIBUTES();
                    secAttribs.nLength = Marshal.SizeOf(typeof(NativeMethods.SECURITY_ATTRIBUTES)); 
                    SafeFileHandle hHCDev = NativeMethods.CreateFile(
                        controllerName,
                        NativeMethods.GENERIC_WRITE,
                        NativeMethods.FILE_SHARE_WRITE,
                        ref secAttribs,
                        NativeMethods.OPEN_EXISTING,
                        0,
                        IntPtr.Zero);

                    // If the handle is valid, then we've successfully opened a Host Controller.
                    if ( !hHCDev.IsInvalid )
                    {
                        String controllerDriver = String.Empty;
                        UInt32 devInst;

                        try
                        {
                            // Obtain the driver key name for this host controller.
                            controllerDriver = GetDeviceDriverKeyName(hHCDev);
                        }
                        catch (Win32Exception e)
                        {
                            // Don't think we need to abort if we don't get the Controller's driver key.
                            // Just make sure it is closed and try the next one.
                            Trace.WriteLine(e.Message);
                        }
                        finally
                        {
                            hHCDev.Close();
                        }

                        if (!String.IsNullOrEmpty(controllerDriver))
                        {
                            // Got the driver key, now get the corresponding DevInst.
                            devInst = DeviceManager.FindDeviceInstance(controllerDriver);

                            if (devInst != 0)
                            {
                                // Add the Controller's DevInst to the set if it is not already there.
                                if (devInstDataSet.ContainsKey(devInst) == false)
                                    devInstDataSet[devInst] = controllerName;
                            }
                        }
                    
                    } // if ( opened controller )
                
                } // for ( 10 controller names )
            
            } // if ( Win2K )

            return devInstDataSet;
        }

        /// <summary>
        /// Gets the Driver key for a USB Host Controller by sending a DeviceIoControl to the
        /// opened device. Worker function for GetDevInstDataSet().
        /// </summary>
        /// <param name="hController"> Handle to a USB controller.</param>
        /// <returns>Driver key name for this host controller.</returns>
        /// <exception cref="System.ComponentModel.Win32Exception">DeviceIoControl() failure.</exception>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Interoperability", "CA1404:CallGetLastErrorImmediatelyAfterPInvoke")]
        private String GetDeviceDriverKeyName(SafeFileHandle hController)
        {
            String driver = String.Empty;

            int nBytesReturned;
            NativeMethods.USB_HCD_DRIVERKEY_NAME controllerDriver = new NativeMethods.USB_HCD_DRIVERKEY_NAME();
            int nBytes = Marshal.SizeOf(controllerDriver);
            IntPtr ptrControllerDriver = Marshal.AllocHGlobal(nBytes);

            // get the Driver Key
            if (NativeMethods.DeviceIoControl(hController, NativeMethods.IOCTL_GET_HCD_DRIVERKEY_NAME, ptrControllerDriver,
                nBytes, ptrControllerDriver, nBytes, out nBytesReturned, IntPtr.Zero))
            {
                // SUCCESS
                controllerDriver = (NativeMethods.USB_HCD_DRIVERKEY_NAME)Marshal.PtrToStructure(ptrControllerDriver, typeof(NativeMethods.USB_HCD_DRIVERKEY_NAME));
                driver = controllerDriver.DriverKeyName;

                Marshal.FreeHGlobal(ptrControllerDriver);
            }
            else
            {
                // FAILED
                int error = Marshal.GetLastWin32Error();
                Marshal.FreeHGlobal(ptrControllerDriver);

                throw new Win32Exception(error);
            }

            return driver;
        }
    }
}