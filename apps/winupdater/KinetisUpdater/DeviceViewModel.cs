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
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using System.Windows.Data;
using System.Windows.Media.Imaging;

using DevSupport.DeviceManager;

namespace KinetisUpdater.ViewModel
{
    //==============================================================================
    //==============================================================================
    /// <summary>
    /// ViewModel for DeviceManager.Device
    /// </summary>
    public class DeviceViewModel : ObservableObject
    {
        private Device Device;
        public enum SerialTransport { None, UART, I2C, SPI }

        public DeviceViewModel() : this(null) { }

        public DeviceViewModel(Device device)
        {
            Device = device;
            if (!IsNull)
            {
                PeripheralConfiguration = new blfwkdll.Peripheral.PeripheralConfigData();
                PeripheralConfiguration.peripheralType = this.PeripheralType;
                if (IsSerial)
                {
                    PeripheralConfiguration.usePing = true;
                    PeripheralConfiguration.comPortName = Port;
                    PeripheralConfiguration.comPortSpeed = 57600;
                    PeripheralConfiguration.serialReadTimeoutMs = 5000;         //TODO: Get this info
                    PeripheralConfiguration.busPalConfig.transport = this.BusPalTransport;
                }
                else if (IsUsbHid)
                {
                    var hid = (HidDevice)this.Device;
                    PeripheralConfiguration.usbHidVid = hid.Vid;
                    PeripheralConfiguration.usbHidPid = hid.Pid;
                    PeripheralConfiguration.serialReadTimeoutMs = 5000;         //TODO: Get this info
                    PeripheralConfiguration.usbHidSerialNumber = String.Empty;  //TODO: Get this info
                }

                // We must initialize this.PeripheralConfiguration before
                // creating the BootloaderViewModel.
                UpdaterModel = new UpdaterViewModel(this);
            }
        }

        public BitmapSource Icon
        {
            get { return Device == null ? null : Device.Icon; }
            set { }
        }

        public override string ToString()
        {
            return IsNull ? "Select a device" : Device.ToString();
        }

        public bool IsNull
        {
            get { return Device == null; }
        }

        public bool IsSerial
        {
            get { return this.ToString().Contains("(COM"); }
        }

        public bool IsUsbHid
        {
            get { return !IsNull && Device is HidDevice; }
        }

        public String Port
        {
            get
            {
                // find COMxx in "(http://www.pemicro.com/opensda)(COM41)"
                return IsSerial ?
                    this.ToString().Substring(this.ToString().LastIndexOf('(') + 1, this.ToString().LastIndexOf(')') - this.ToString().LastIndexOf('(') - 1) :
                    String.Empty;
            }
        }

        public String DeviceInstance { get { return this.IsNull ? String.Empty : Device.DeviceInstanceId; } }

        public SerialTransport Transport
        {
            get { return this.IsSerial ? _Transport : SerialTransport.None; }
            set
            {
                _Transport = value;
                PeripheralConfiguration.busPalConfig.transport = BusPalTransport;
                PeripheralConfiguration.peripheralType = PeripheralType;
                RaisePropertyChangedEvent("Transport");
                RaisePropertyChangedEvent("PeripheralType");
                RaisePropertyChangedEvent("BusPalTransport");
            }
        }
        private SerialTransport _Transport = SerialTransport.UART;

        public blfwkdll.Peripheral.PeripheralConfigData PeripheralConfiguration { get; private set; }

        private blfwkdll.Peripheral.HostPeripheralTypes PeripheralType
        {
            get
            {

                if (this.IsNull)
                {
                    return blfwkdll.Peripheral.HostPeripheralTypes.None;
                }
                else if (this.IsSerial && this.Transport == SerialTransport.UART)
                {
                    return blfwkdll.Peripheral.HostPeripheralTypes.UART;
                }
                else if (this.IsSerial)
                {
                    return blfwkdll.Peripheral.HostPeripheralTypes.BusPal_UART;
                }
                else
                {
                    return blfwkdll.Peripheral.HostPeripheralTypes.USB_HID;
                }
            }
        }

        private blfwkdll.BusPal.BusPalTransports BusPalTransport
        {
            get
            {
                switch (this.Transport)
                {
                    case SerialTransport.UART:
                        return blfwkdll.BusPal.BusPalTransports.None;
                    case SerialTransport.I2C:
                        return blfwkdll.BusPal.BusPalTransports.I2C;
                    case SerialTransport.SPI:
                        return blfwkdll.BusPal.BusPalTransports.SPI;
                    default:
                        return blfwkdll.BusPal.BusPalTransports.None;
                }
            }
        }

        public UpdaterViewModel UpdaterModel { get; private set; }

    } // class DeviceViewModel

    public class TransportConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value is DeviceViewModel.SerialTransport)
            {
                switch ((DeviceViewModel.SerialTransport)value)
                {
                    case DeviceViewModel.SerialTransport.I2C:
                        return "I2C using BusPal";
                    case DeviceViewModel.SerialTransport.SPI:
                        return "SPI using BusPal";
                    default:
                        break;
                }
            }
            return value.ToString();
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value is string)
                return Enum.Parse(typeof(DeviceViewModel.SerialTransport), (value as string).Split(' ')[0]);
            else
                return (DeviceViewModel.SerialTransport)((System.Windows.FrameworkElement)value).Tag;
        }
    }

    //==============================================================================
    //==============================================================================
    /// <summary>
    /// ViewModel for DeviceManager.DeviceManager
    /// </summary>
    public class DeviceManagerViewModel : ObservableObject
    {
        public UInt32[] UartBaudRates { get { return _UartBaudRates; } }
        private UInt32[] _UartBaudRates = { /*75, 110, 134, 150, 300, 600, 1200, 1800, 2400, 4800, 7200,*/ 9600, 14400, 19200, 38400, 57600, 115200, 128000 };
        public ImageFileManagerViewModel ImageFileManager { get; private set; }
        public ObservableCollection<DeviceViewModel> DeviceViewModelList { get; private set; }
        private DeviceViewModel _PreviousDevice = null;
        private DeviceViewModel _CurrentDevice = new DeviceViewModel();
        public DeviceViewModel CurrentDevice
        {
            get { return _CurrentDevice; }
            set
            {
                if (_CurrentDevice == value)
                    return;

                // If the previous CurrentDevice was not null, we need to 
                // Dispose of it's Bootloader to send a delayed ACK if
                // required by the protocol.
                if (!_CurrentDevice.IsNull)
                {
                    if (_CurrentDevice.UpdaterModel.IsConnected)
                    {
                        _CurrentDevice.UpdaterModel.Updater.reset();
                    }
                    if ( _CurrentDevice.IsSerial )
                        _CurrentDevice.UpdaterModel.IsConnectedTimer.Stop();
                    _CurrentDevice.UpdaterModel.Close();
                }

                // Set the _CurrentDevice to the new device.
                if (value == null)
                {
                    _CurrentDevice = new DeviceViewModel();
                }
                else
                {
                    _CurrentDevice = value;
                }

                // If it is not a "null device", see if it has a running bootloader.
                if (!_CurrentDevice.IsNull)
                {
                    // Save Current device for next app launch.
                    Properties.Settings.Default.LastDeviceInstance = _CurrentDevice.DeviceInstance;
                    
                    // See if it has a running bootloader.
                    if ((_CurrentDevice.IsUsbHid && _CurrentDevice.UpdaterModel.AutoConnectUSBDevice) || _CurrentDevice.IsSerial)
                    {
                        _CurrentDevice.UpdaterModel.Ping();
                    }

                    if (_CurrentDevice.IsSerial)
                    {
                        // Start the timer to keep tabs on if the serial device is still running bootloader.
                        _CurrentDevice.UpdaterModel.IsConnectedTimer.Start();
                    }
                }

                RaisePropertyChangedEvent("CurrentDevice");
            }
        }

        public DeviceManagerViewModel()
        {
            // Create an instance of the DeviceManager
            try
            {
                DeviceManager.Instance.DeviceChanged += new DeviceManager.DeviceChangedEventHandler(DeviceManager_DeviceChanged);
                // DeviceManager.Instance.DeviceChangedFiltered += new DeviceManager.DeviceChangedEventFiltered(OnDeviceChangedNotify);
            }
            catch (Exception err)
            {
                Trace.WriteLine(err.Message);
            }

            ImageFileManager = new ImageFileManagerViewModel();
            DeviceViewModelList = new ObservableCollection<DeviceViewModel>();
            InitDeviceList();
            SetCurrentDevice(Properties.Settings.Default.LastDeviceInstance);
        }

        private void InitDeviceList()
        {
            CurrentDevice = new DeviceViewModel();

            foreach (var dev in HidDeviceClass.Instance.Devices)
                DeviceViewModelList.Add(new DeviceViewModel(dev));

            foreach (var dev in SerialPortDeviceClass.Instance.Devices)
                DeviceViewModelList.Add(new DeviceViewModel(dev));
        }

        public bool SetCurrentDevice(String deviceInstance)
        {
            foreach (var model in DeviceViewModelList)
            {
                if (model.DeviceInstance == deviceInstance)
                {
                    CurrentDevice = model;
                    return true;
                }
            }
            return false;
        }

        void DeviceManager_DeviceChanged(DeviceChangedEventArgs eventArgs)
        {
            Trace.WriteLine(String.Format("*** MainWindow.DeviceManager_DeviceChanged(): {0}: {1}, {2}({3})",
                eventArgs.Event, eventArgs.DeviceId, Thread.CurrentThread.Name, Thread.CurrentThread.GetHashCode()));
            String logStr = eventArgs.Event + ": " + eventArgs.DeviceId + "\r\n"; ;
            //LogTextBox.AppendText(logStr);

            //TODO: Fix Device Arrival/Removal messages coming in the wrong order!

            if (eventArgs.Event == DeviceChangeEvent.DeviceArrival)
            {
                DeviceManager.Refresh();
                if (CurrentDevice.IsNull && _PreviousDevice != null && _PreviousDevice.DeviceInstance == eventArgs.DeviceId)
                {
                    DeviceViewModelList.Add(_PreviousDevice);
                    CurrentDevice = _PreviousDevice;
                    _PreviousDevice = null;
                }
                else
                {
                    var device = DeviceManager.DeviceClassMgrs.SelectMany(mgr => mgr.Devices).Single(dev => dev.DeviceInstanceId == eventArgs.DeviceId);
                    DeviceViewModelList.Add(new DeviceViewModel(device));
                }
            }
            else if (eventArgs.Event == DeviceChangeEvent.DeviceRemoval)
            {
                if (CurrentDevice.DeviceInstance == eventArgs.DeviceId)
                {
                    _PreviousDevice = CurrentDevice;
                    CurrentDevice = new DeviceViewModel();
                }

                var devModel = DeviceViewModelList.SingleOrDefault(model => model.DeviceInstance == eventArgs.DeviceId);
                DeviceViewModelList.Remove(devModel);

                DeviceManager.Refresh();
            }

//            else if (MainMenuItem_File_TrackUsbPort.Checked)
//            {
//                _CurrentDevice = FindDeviceByPort(LastPort);
//                CurrentApi = LoadDeviceApis(_CurrentDevice);
//
//                if (_CurrentDevice != null)
//                {
//                    logStr = String.Format("Track USB Port(Hub{0},Port{1}): AutoSelect \"{2}\".\r\n", _CurrentDevice.UsbHub.Index, _CurrentDevice.UsbPort, _CurrentDevice.ToString());
//                    LogTextBox.AppendText(logStr);
//                }
//            }

//                        UpdateStatus();
        }
    } // class DeviceManagerViewModel

}

