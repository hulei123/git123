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
using System.IO;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace DevSupport.DeviceManager
{
    /// <summary>
    /// A (COM)Port device.
    /// </summary>
    sealed public partial class SerialPortDevice : Device//, IComparable
    {
        internal SerialPortDevice(/*DeviceClass deviceClass,*/ UInt32 deviceInstance, string path/*, int index*/)
            : base(/*deviceClass,*/ deviceInstance, path/*, index*/)
        { }

        private String _Port;

        /// <summary>
        /// Represent the device by it's FriendlyName
        /// ex. ex. "Communications Port (COM1)"
        /// </summary>
        public override string ToString()
        {
            return FriendlyName;
        }

        /// <summary>
        /// Gets the Serial Port Name. ex. "COM1"
        /// </summary>
        public String Port()
        {
            if (_Port == null)
            {
                // ex. "Communications Port (COM1)"
                var start = FriendlyName.IndexOf('(') + 1;
                var end = FriendlyName.IndexOf(')');
                _Port = FriendlyName.Substring(start, end - start);
            }

            return _Port;
        }

    } // class SerialPortDevice


}
