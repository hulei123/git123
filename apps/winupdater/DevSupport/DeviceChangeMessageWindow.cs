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
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Security.Permissions;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace DevSupport.DeviceManager
{
    /// <summary>
    /// Type of device change event.
    /// </summary>
    public enum DeviceChangeEvent
    {
        Unknown,
        DeviceArrival,
        DeviceRemoval,
        VolumeArrival,
        VolumeRemoval,
        HubArrival,
        HubRemoval,
        PortArrival,
        PortRemoval
    }

    /// <summary>
    /// Monitors WM_DEVICECHANGE native Windows messages and asynchronously invokes DeviceChangedMsg event
    /// for the following event types: USB Device Arrival/Removal, Volume Arrival/Removal, and USB Hub Arrival/Removal.
    /// </summary>
    public class DeviceChangeWindow : NativeWindow, IDisposable
    {
        internal delegate void DeviceChangedMsgHandler(DeviceChangeEvent devEvent, String devDetails);
        internal event DeviceChangedMsgHandler DeviceChangedMsg;

        private IntPtr _UsbDevNotifyHandle;
        private IntPtr _UsbHubNotifyHandle;
        
        public DeviceChangeWindow()
        {
            /// Make WndProc() start getting messages.
            CreateParams createParams = new CreateParams();
            createParams.ClassName = "STATIC";
            CreateHandle(createParams);
        
            /// Register for usb devices notifications.
            NativeMethods.DEV_BROADCAST_DEVICEINTERFACE_NONAME broadcastInterface = new NativeMethods.DEV_BROADCAST_DEVICEINTERFACE_NONAME();
            broadcastInterface.dbcc_devicetype = NativeMethods.DBT_DEVTYP.DEVICEINTERFACE;
            broadcastInterface.dbcc_classguid = NativeMethods.GUID_DEVINTERFACE_USB_DEVICE;
            IntPtr buffer = Marshal.AllocHGlobal(new IntPtr(broadcastInterface.dbcc_size));
            Marshal.StructureToPtr(broadcastInterface, buffer, false);
            _UsbDevNotifyHandle = NativeMethods.RegisterDeviceNotification(Handle, buffer, NativeMethods.DEVICE_NOTIFY_WINDOW_HANDLE);

            /// Register for usb hub notifications.
            broadcastInterface.dbcc_classguid = NativeMethods.GUID_DEVINTERFACE_USB_HUB;
            Marshal.StructureToPtr(broadcastInterface, buffer, true);
            _UsbHubNotifyHandle = NativeMethods.RegisterDeviceNotification(Handle, buffer, NativeMethods.DEVICE_NOTIFY_WINDOW_HANDLE);

            Marshal.FreeHGlobal(buffer);
        }

        #region IDisposable Implemetation

        /// <summary>
        /// Used to determine if Dispose() has already been called.
        /// </summary>
        private bool _Disposed = false;

        /// <summary>
        /// Finalizer calls Dispose(false) worker function.
        /// </summary>
        ~DeviceChangeWindow()
        {
            // Call our helper method. 
            // Specifying "false" signifies that the GC triggered the clean up.
            Dispose(false);
        }

        /// <summary>
        /// IDisposable Implemetation.
        /// </summary>
        public void Dispose()
        {
            // Call our helper method. 
            // Specifying "true" signifies that the object user triggered the clean up.
            Dispose(true);

            // Now suppress finalization.
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Dispose Pattern worker function.
        /// </summary>
        /// <param name="disposing">false - called from Finalizer, true - called from Dispose()</param>
        private void Dispose(bool disposing)
        {
            // Be sure we have not already been disposed!
            if (!_Disposed)
            {
                // If disposing equal true, dispose all managed resources.
                if (disposing)
                {
                    // Dispose managed resources.
                }
                
                // Clean up unmanaged resources here.
                
                if (_UsbDevNotifyHandle != IntPtr.Zero)
                {
                    bool ret = NativeMethods.UnregisterDeviceNotification(_UsbDevNotifyHandle);
                    _UsbDevNotifyHandle = IntPtr.Zero;
                }

                if (_UsbHubNotifyHandle != IntPtr.Zero)
                {
                    bool ret = NativeMethods.UnregisterDeviceNotification(_UsbHubNotifyHandle);
                    _UsbHubNotifyHandle = IntPtr.Zero;
                }

                // A window is not eligible for garbage collection when it is associated with
                // a window handle. To ensure proper garbage collection, handles must either be
                // destroyed manually using DestroyHandle or released using ReleaseHandle.
                DestroyHandle();
            }
            _Disposed = true;
        }

        /// <summary>
        /// 
        /// </summary>

        #endregion

        /// <summary>
        /// Monitor WM_DEVICECHANGE messages.
        /// </summary>
        /// <param name="msg">Native Windows message.</param>
        protected override void WndProc(ref Message msg)
        {
            if (msg.Msg == NativeMethods.WM_DEVICECHANGE)
            {
                OnDeviceChange(msg);
            }
            base.WndProc(ref msg);
        }

        /// <summary>
        /// Worker function for WM_DEVICECHANGE messages. Invokes DeviceChangedMsg event.
        /// </summary>
        /// <param name="msg">Native Windows message - WM_DEVICECHANGE</param>
        private void OnDeviceChange(Message msg)
        {
            DeviceChangeEvent devEvent = DeviceChangeEvent.Unknown;
            String devDetails = String.Empty;

            if (msg.LParam != IntPtr.Zero)
            {
                NativeMethods.DEV_BROADCAST_HDR db = (NativeMethods.DEV_BROADCAST_HDR)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_HDR));

                switch (msg.WParam.ToInt32())
                {
                    case NativeMethods.DBT_DEVICEARRIVAL:
                        if (db.dbch_devicetype == NativeMethods.DBT_DEVTYP.DEVICEINTERFACE)
                        {
                            NativeMethods.DEV_BROADCAST_DEVICEINTERFACE dbdi = (NativeMethods.DEV_BROADCAST_DEVICEINTERFACE)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_DEVICEINTERFACE));
                            if (dbdi.dbcc_classguid == NativeMethods.GUID_DEVINTERFACE_USB_DEVICE)
                            {
                                devEvent = DeviceChangeEvent.DeviceArrival;
                                devDetails = dbdi.dbcc_name;
                            }
                            else if (dbdi.dbcc_classguid == NativeMethods.GUID_DEVINTERFACE_USB_HUB)
                            {
                                devEvent = DeviceChangeEvent.HubArrival;
                                devDetails = dbdi.dbcc_name;
                            }
                        }
                        else if (db.dbch_devicetype == NativeMethods.DBT_DEVTYP.VOLUME)
                        {
                            NativeMethods.DEV_BROADCAST_VOLUME dbv = (NativeMethods.DEV_BROADCAST_VOLUME)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_VOLUME));
                            devEvent = DeviceChangeEvent.VolumeArrival;
                            devDetails = DrivesFromMask(dbv.dbcv_unitmask);
                        }
                        else if (db.dbch_devicetype == NativeMethods.DBT_DEVTYP.PORT)
                        {
                            NativeMethods.DEV_BROADCAST_PORT dbp = (NativeMethods.DEV_BROADCAST_PORT)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_PORT));
                            devEvent = DeviceChangeEvent.PortArrival;
                            devDetails = dbp.dbcp_name;
                        }
                        break;
                    case NativeMethods.DBT_DEVICEREMOVECOMPLETE:
                        if (db.dbch_devicetype == NativeMethods.DBT_DEVTYP.DEVICEINTERFACE)
                        {
                            NativeMethods.DEV_BROADCAST_DEVICEINTERFACE dbdi = (NativeMethods.DEV_BROADCAST_DEVICEINTERFACE)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_DEVICEINTERFACE));

                            if (dbdi.dbcc_classguid == NativeMethods.GUID_DEVINTERFACE_USB_DEVICE)
                            {
                                devEvent = DeviceChangeEvent.DeviceRemoval;
                                devDetails = dbdi.dbcc_name;
                            }
                            else if (dbdi.dbcc_classguid == NativeMethods.GUID_DEVINTERFACE_USB_HUB)
                            {
                                devEvent = DeviceChangeEvent.HubRemoval;
                                devDetails = dbdi.dbcc_name;
                            }
                        }
                        else if (db.dbch_devicetype == NativeMethods.DBT_DEVTYP.VOLUME)
                        {
                            NativeMethods.DEV_BROADCAST_VOLUME dbv = (NativeMethods.DEV_BROADCAST_VOLUME)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_VOLUME));
                            devEvent = DeviceChangeEvent.VolumeRemoval;
                            devDetails = DrivesFromMask(dbv.dbcv_unitmask);
                        }
                        else if (db.dbch_devicetype == NativeMethods.DBT_DEVTYP.PORT)
                        {
                            NativeMethods.DEV_BROADCAST_PORT dbp = (NativeMethods.DEV_BROADCAST_PORT)Marshal.PtrToStructure(msg.LParam, typeof(NativeMethods.DEV_BROADCAST_PORT));
                            devEvent = DeviceChangeEvent.PortRemoval;
                            devDetails = dbp.dbcp_name;
                        }
                        break;
                    default:
                        Trace.Assert(false, "Invalid msg.WParam.");
                        break;
                } // end switch (nEventType)

                Trace.WriteLine(String.Format("*** DeviceChangeWindow.OnDeviceChange(), {0}, {1}, {2}({3})", devEvent, devDetails, Thread.CurrentThread.Name, Thread.CurrentThread.GetHashCode()));

                // let's figure out what to do with the WM_DEVICECHANGE message
                // after we get out of this loop so we don't miss any messages.
                if (DeviceChangedMsg != null)
                {
                    DeviceChangedMsg.BeginInvoke(devEvent, devDetails, null, null);
                }

            } // end if (lpdb)

            msg.Result = new IntPtr(1); // true
            return;
        }

        /// <summary>
        /// Worker function for OnDeviceChange() to get drive letters from the bitmask.
        /// </summary>
        /// <param name="UnitMask">Logical unit mask identifying one or more logical units. Each bit in the mask corresponds to one logical drive. Bit 0 represents drive A, bit 1 represents drive B, and so on.</param>
        /// <returns>String representing the drives identified in the UnitMask parameter in the form of "F" or "FG" for multiple drives.</returns>
        private static String DrivesFromMask(uint UnitMask)
        {
            String DriveStr = "";
            Char DriveCh;

            for (DriveCh = 'A'; DriveCh <= 'Z'; ++DriveCh, UnitMask >>= 1)
            {
                if ((UnitMask & 1) == 1)
                    DriveStr += DriveCh;
            }
            return DriveStr;
        }
    
    } // class DeviceChangeWindow

} // namespace DevSupport.DeviceManager
