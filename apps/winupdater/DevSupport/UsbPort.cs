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
    public class UsbPort : IFormattable
	{
        /// <summary>
        /// The hub to which this port belongs;
        /// </summary>
        private UsbHub _ParentHub;
        private int _Index;
        private String _DriverName;
        private Device _AttachedDevice;
        private NativeMethods.USB_NODE_CONNECTION_INFORMATION_EX _NodeConnectionInfo;


        [TypeConverter(typeof(ExpandableObjectConverter))]
        public NativeMethods.USB_NODE_CONNECTION_INFORMATION_EX NodeConnectionInfo
	    {
		    get 
            {
                return _NodeConnectionInfo;
            }
            // Set in this.Refresh()
            private set
            {
                _NodeConnectionInfo = value;
            }
	    }

        public String DriverName
        {
            get { return _DriverName; }
            // Set in this.Refresh()
            private set { _DriverName = value; }
        }

        /// <summary>
		/// The 1-based index of the port on the hub.
		/// </summary>
	    public int Index
	    {
		    get { return _Index;}
	    }

        /// <summary>
        /// The USB hub that owns this port.
        /// </summary>
        public UsbHub Hub
        {
            get { return _ParentHub; }
        }

        // TODO: add a format "short" defined as (device.UsbHub + "Port " + device.UsbPort) or simply ( Hub X Port Y)
        public string ToString(String format, IFormatProvider formatProvider)
        {
            String str = null;

            if (format == "P")
            {
                // Format simply as Port X
                str = String.Format("Port {0}", Index);
            }
            else if (format == "ID")
            {
                // Format as Hub X Port Y
                str = String.Format("Hub {0} Port {1}", this.Hub.Index, this.Index);
            }
            else if (format == "D")
            {
                // Format as Port X - Device.ToString()
                str = String.Format("Port {0} - {1}", Index, AttachedDevice.ToString());
            }
            else if (format == "H")
            {
                // If the connection is to a hub, format Port X - Hub Y
                UsbHub hub = AttachedDevice as UsbHub;
                str = String.Format("Port {0} - Hub {1}", Index, hub.Index);
            }
            else if (format == "E")
            {
                // Format as Port X [ConnectionStatus] Device.ToString()
                str = String.Format("Port {0} [{1}] {2}", Index, NodeConnectionInfo.ConnectionStatus, AttachedDevice.ToString());
            }
            else if (format == "U")
            {
                // Format as Port X [ConnectionStatus] UNKNOWN
                str = String.Format("Port {0} [{1}] UNKNOWN", Index, NodeConnectionInfo.ConnectionStatus);
            }
            else
            {
                str = ToString();
            }

            return str;
        }

        public override string ToString()
        {
            String str;

            // If we have Connection information about the port.
            if (NodeConnectionInfo != null)
            {
                // If the port is not connected to a device
                if (NodeConnectionInfo.ConnectionStatus == NativeMethods.USB_CONNECTION_STATUS.NoDeviceConnected)
                {
                    // Format simply as Port X
                    str = this.ToString("P", null);
                }
                // Else the port is reportedly connected to a device
                else
                {
                    // If we really have a device object
                    if (AttachedDevice != null)
                    {
                        // And the ConnectionStatus is not an error
                        if (NodeConnectionInfo.ConnectionStatus == NativeMethods.USB_CONNECTION_STATUS.DeviceConnected)
                        {
                            // If the connection is to a hub, format Port X - UsbHub Y
                            if (NodeConnectionInfo.DeviceIsHub != 0)
                            {
                                UsbHub hub = AttachedDevice as UsbHub;
                                if (hub != null)
                                {
                                    str = this.ToString("H", null);
                                }
                                else
                                {
                                    str = String.Format("Port {0} - UsbHub <null>", Index);
                                    throw new ArgumentNullException("hub");
                                }
                            }
                            else
                            {
                                // Format as Port X - Device.ToString()
                                str = this.ToString("D", null);
                            }
                        }
                        // Else the ConnectionStatus is an error, so show it
                        else
                        {
                            // Format as Port X [ConnectionStatus] Device.ToString()
                            str = this.ToString("E", null);
                        }
                    }
                    // Else we don't really have a device object so we report ConnectionStatus and UNKNOWN
                    else
                    {
                        // Format as Port X [ConnectionStatus] UNKNOWN
                        str = this.ToString("U", null);
                    }
                }
            }
            // Else we don't have Connection information about the port. 
            else
            {
                // Probably an error condition.
                str = String.Format("Port {0} - NO INFO!", Index);
            }

            return str;
        }

        /// <summary>
        /// Create a UsbPort instance.
        /// </summary>
        /// <param name="hub">The hub to which this port belongs</param>
        /// <param name="index">The 1-based index of the port on the hub.</param>
        public UsbPort(UsbHub hub, int index)
	    {
            _ParentHub = hub;
            _Index = index;

            this.Refresh();
	    }

        /// <summary>
        /// Gets the USB device connected to the USB Port or null if not connected.
        /// </summary>
        public Device AttachedDevice
        {
            get 
            {
                if (_AttachedDevice == null)
                {
                    if (NodeConnectionInfo.DeviceIsHub != 0)
                    {
                        _AttachedDevice = FindHub(DriverName);
                    }
                    else
                    {
                        _AttachedDevice = DeviceManager.FindChildDevice(DriverName);
                    }
                }
                return _AttachedDevice; 
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Interoperability", "CA1404:CallGetLastErrorImmediatelyAfterPInvoke")]
        public int Refresh()
        {
            int error = 0;

            // Reset Port Properties
            if (_AttachedDevice != null)
            {
                _AttachedDevice.Dispose();
                _AttachedDevice = null;
            }
            DriverName = String.Empty;

            // Open parent hub
            SafeFileHandle hHub = _ParentHub.Open();
            if ( hHub.IsInvalid )
            {
                error = Marshal.GetLastWin32Error();
                Trace.WriteLine(String.Format("*** ERROR 0x{0:X} ({0}): UsbPort.Refresh() hub: {1} port: {2}", error, _ParentHub.Index, Index));
                return error;
            }

            // Get Connection Information
            NodeConnectionInfo = _ParentHub.GetNodeConnectionInfo(hHub, Index);
            if (NodeConnectionInfo == null)
            {
                error = Marshal.GetLastWin32Error();
                hHub.Close();
                Trace.WriteLine(String.Format("*** ERROR 0x{0:X} ({0}): UsbPort.Refresh() hub: {1} port: {2}", error, _ParentHub.Index, Index));
                return error;
            }

            //
            // There is a device connected to this Port
            //
            if ( NodeConnectionInfo.ConnectionStatus == NativeMethods.USB_CONNECTION_STATUS.DeviceConnected )
            {
                // Get the name of the driver key of the device attached to the specified port.
                DriverName = _ParentHub.GetNodeConnectionDriver(hHub, Index);
                if ( String.IsNullOrEmpty(DriverName) ) 
                {
                    error = Marshal.GetLastWin32Error();
                    hHub.Close();
                    Trace.WriteLine(String.Format("*** ERROR 0x{0:X} ({0}): UsbPort.Refresh() failed. H:{1}, P{2}", error, _ParentHub.Index, Index));
                    return error;
                }

            } // end if(connected)

            // Close the hub
            hHub.Close();

            Trace.WriteLine(String.Format("UsbPort.Refresh() - Hub {0} Port {1}", _ParentHub.Index, Index));

            return error;
        }

        private UsbHub FindHub(String driverName)
        {
            if (String.IsNullOrEmpty(driverName))
            {
                return null;
            }

            UsbHub hub = UsbHubClass.Instance.FindDeviceByDriver(driverName) as UsbHub;

	        return hub;
        }


    } // class UsbPort
		 
} // namespace DevSupport.DeviceManager
