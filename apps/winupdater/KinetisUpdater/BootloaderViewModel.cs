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
using System.Diagnostics;
using System.Threading;
using System.Windows;
using System.Windows.Input;
using System.Windows.Threading;


namespace KinetisUpdater.ViewModel
{
    /// <summary>
    /// ViewModel for blfwk.Bootloader
    /// </summary>
    public class BootloaderViewModel : ObservableObject
    {
        protected DeviceViewModel DeviceModel { get; set; }
        public blfwkdll.Updater Updater { get; private set; }

        public BootloaderViewModel(DeviceViewModel device)
        {
            DeviceModel = device;
            // Creating a Bootloader object entails Pinging the peripheral.
            // We don't want to talk to the device until it becomes selected.
            // That way, we will not try to talk to devices that are not
            // known to be Kinestis Bootloader devices, and we will not 
            // shut down alternate peripherals on a given Kinetis Bootloader
            // device.
            Updater = null;

            if (DeviceModel.IsSerial)
            {
                IsConnectedTimer = new System.Windows.Threading.DispatcherTimer();
                IsConnectedTimer.Tick += new EventHandler(IsConnectedTimer_Tick);
                IsConnectedTimer.Interval = new TimeSpan(0, 0, 1);
            }
        }

        private void IsConnectedTimer_Tick(object sender, EventArgs e)
        {
            Ping();
            if (IsConnectedTimer.Interval.Seconds > 1)
            {
                IsConnectedTimer.Stop();
                IsConnectedTimer.Interval = new TimeSpan(0, 0, 1);
                IsConnectedTimer.Start();
            }
        }

        // Close the native bootloader object.
        public void Close()
        {
            if (Updater != null)
            {
                Updater.Dispose();
                Updater = null;
                IsConnected = false;
            }
        }

        public DispatcherTimer IsConnectedTimer { get; private set; }

        private Boolean _IsConnected = false;
        public Boolean IsConnected
        {
            get { return _IsConnected; }
            set
            {
                if (_IsConnected != value)
                {
                    _IsConnected = value;
                    RaisePropertyChangedEvent("IsConnected");
                    RaisePropertyChangedEvent("Version");
                    RaisePropertyChangedEvent("SerialProtocolVersion");
                    RaisePropertyChangedEvent("SecurityState");
                }
            }
        }

        public Boolean isFlashEraseAllUnsecureSupported
        {
            get { return Updater.IsCommandSupported("flash-erase-all-unsecure"); }
        }

        public String SerialProtocolVersion
        {
            get
            {
                if (DeviceModel.IsSerial && IsConnected)
                    return this.Updater.SerialProtocolVersion.ToString();
                else
                    return String.Empty;
            }
        }

        public Boolean InSecurity
        {
            get { return _SecurityState.ToBoolean(); }
        }

        private blfwkdll.SecurityState _SecurityState;
        public String SecurityState
        {
            get
            {
                if (IsConnected)
                {
                    _SecurityState = this.Updater.TargetSecurityState;
                    RaisePropertyChangedEvent("InSecurity");
                    if (InSecurity)
                        RaisePropertyChangedEvent("isFlashEraseAllUnsecureSupported");
                    return _SecurityState.ToString();
                }
                else
                    return String.Empty;
            }
        }

        public String Version
        {
            get
            {
                if (IsConnected)
                    return this.Updater.Version.ToString();
                else
                    return String.Empty;
            }
        }

        private String _LastError = String.Empty;
        public String LastError
        {
            get { return _LastError; }
            set
            {
                if (_LastError != value)
                {
                    _LastError = value;
                    RaisePropertyChangedEvent("LastError");
                }
            }
        }

        internal Boolean Ping()
        {
            if (DeviceModel.IsNull)
            {
                LastError = "The device does not support a bootloader.";
                IsConnected = false;
                return false;
            }

            if (Updater == null)
            {
                try
                {
                    Updater = new blfwkdll.Updater(DeviceModel.PeripheralConfiguration);

                    LastError = "Success.";
                    IsConnected = true;
                    return true;
                }
                catch (Exception e)
                {
                    LastError = e.Message;
                    IsConnected = false;
                    return false;
                }
            }

            try
            {
                Updater.ping(0, 0, 0);
                LastError = "Success.";
                IsConnected = true;
                return true;
            }
            catch (Exception e)
            {
                LastError = e.Message;
                Close();
                return false;
            }
        }

        private Boolean _AutoConnectUSBDevice = true;
        public Boolean AutoConnectUSBDevice
        {
            get { return _AutoConnectUSBDevice; }
            set
            {
                _AutoConnectUSBDevice = value;
            }
        }

        public ICommand ConnectCommand
        {
            get { return new DelegateCommand(ConnectButtonHandler); }
        }

        protected void ConnectButtonHandler()
        {
            if (DeviceModel.IsSerial && !IsConnectedTimer.IsEnabled)
                IsConnectedTimer.Start();
            if (DeviceModel.IsUsbHid)
                AutoConnectUSBDevice = true;
            Ping();
        }

        public ICommand ResetCommand
        {
            get { return new DelegateCommand(ResetButtonHandler); }
        }

        protected void ResetButtonHandler()
        {
            // If the User clicked the Reset button on the Device Configuration page, 
            // then we should stop the connection polling to allow the User to choose
            // a different peripheral on the same device.
            if (DeviceModel.IsSerial)
                IsConnectedTimer.Stop();
            if (DeviceModel.IsUsbHid)
                AutoConnectUSBDevice = false;
            Reset();
        }

        protected void Reset()
        {
            Updater.reset();
            Close();
        }

        public ICommand ChangeBaudRateCommand
        {
            get { return new DelegateCommand(ChangeBaudRate); }
        }

        private void ChangeBaudRate()
        {
            if (this.IsConnected)
            {
                Reset();
                Thread.Sleep(10);
                Ping();
            }
        }


    }
}

