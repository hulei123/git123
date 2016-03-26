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

//#define TEST_WINDOW_LOGIC

using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;

namespace KinetisUpdater.ViewModel
{
    //==============================================================================
    //==============================================================================
    /// <summary>
    /// ViewModel for blfwkdll.Updater
    /// </summary>
    public class UpdaterViewModel : BootloaderViewModel
    {
        public class UpdaterAsyncCommand : AsyncCommand<object>
        {
            public UpdaterAsyncCommand(Func<CancellationToken, Task> command)
                : base(async token => { await command(token); return null; }) { }

            // Update can only run once.
            public override bool CanExecute(object parameter)
            {
                return Execution == null || Execution.IsCompleted;
            }
        }

        public ImageFileViewModel ImageFileModel { get; set; }
//        private blfwkdll.Updater.UpdaterOperationProgressDelegate ProgressCallback = 
//            new blfwkdll.Updater.UpdaterOperationProgressDelegate(UpdaterProgress);

        public UpdaterViewModel(DeviceViewModel device)
            : base(device)
        {
            ImageFileModel = null;
            var progress = new Progress<blfwkdll.Updater.UpdaterOperationProgressData>(UpdaterProgress);
            UpdateCommand = new UpdaterAsyncCommand(token => Update(this, progress, token));

            BackdoorKey = Properties.Settings.Default.BackdoorKey;

            Status = "Ready";
            Value = 0;
            Total = 100;
        }


        private Boolean _FlashEraseAllUnsecure = false;
        public Boolean FlashEraseAllUnsecure
        {
            get { return _FlashEraseAllUnsecure; }
            set 
            {
                if (value != _FlashEraseAllUnsecure)
                {
                    _FlashEraseAllUnsecure = value;
                    RaisePropertyChangedEvent("FlashEraseAllUnsecure");
                }
            }
        }

        private Boolean _UnlockWithBackdoorKey = true;
        public Boolean UnlockWithBackdoorKey
        {
            get { return _UnlockWithBackdoorKey; }
            set
            {
                if (value != _UnlockWithBackdoorKey)
                {
                    _UnlockWithBackdoorKey = value;
                    RaisePropertyChangedEvent("UnlockWithBackdoorKey");
                }
            }
        }

        private String _BackdoorKey;
        public String BackdoorKey
        {
            get { return _BackdoorKey; }
            set
            {
                if (value != _BackdoorKey)
                {
                    _BackdoorKey = value;
                    Properties.Settings.Default.BackdoorKey = _BackdoorKey;
                    RaisePropertyChangedEvent("BackdoorKey");
                }
            }
        }

        private String _Status;
        public String Status
        {
            get { return _Status; }
            private set
            {
                if (value != _Status)
                {
                    _Status = value;
                    RaisePropertyChangedEvent("Status");
                }
            }
        }

        private Int32 _Total;
        public Int32 Total
        {
            get { return _Total; }
            private set
            {
                if (value != _Total)
                {
                    _Total = value;
                    RaisePropertyChangedEvent("Total");
                }
            }
        }

        private Int32 _Value;
        public Int32 Value
        {
            get { return _Value; }
            private set
            {
                if (value != _Value)
                {
                    _Value = value;
                    RaisePropertyChangedEvent("Value");
                }
            }
        }

        public ICommand CheckKeyInput
        {
            get
            {
                return new DelegateCommand<KeyEventArgs>((e) =>
                {
                    e.Handled = KeyInputFilter.Filter(e.Key, e.KeyboardDevice.Modifiers) || KeyInputFilter.FilterX(e.Key);
                }
                );
            }
        }

        public IAsyncCommand UpdateCommand { get; private set; }

        private async Task Update(UpdaterViewModel updaterModel, 
            IProgress<blfwkdll.Updater.UpdaterOperationProgressData> progress, 
            CancellationToken token = new CancellationToken())
        {

#if (TEST_WINDOW_LOGIC)

            blfwkdll.Updater.UpdaterOperationProgressData dummyOpData = 
                new blfwkdll.Updater.UpdaterOperationProgressData(new blfwkdll.Updater.UpdaterEnum(0, "Updating..."));
            dummyOpData.Tasks.Add(new blfwkdll.Updater.UpdaterTask(new blfwkdll.Updater.UpdaterEnum(0, "Erasing..."), 0, 100));
            dummyOpData.Tasks.Add(new blfwkdll.Updater.UpdaterTask(new blfwkdll.Updater.UpdaterEnum(1, "Writing..."), 0, 100));

            for (int j = 0; j < dummyOpData.Tasks.Count; ++j)
            {
                dummyOpData.CurrentTaskIndex = j;
                if ( progress != null )
                    progress.Report(dummyOpData);

                for (UInt32 i = 10; i <= dummyOpData.Tasks[j].Total; i += 10)
                {
                    dummyOpData.Tasks[j].CurrentPosition = i;
                    if (progress != null)
                        progress.Report(dummyOpData);

                    await Task.Delay(TimeSpan.FromSeconds(.1), token).ConfigureAwait(false);

                    if (j == 1 && i == 20)
                        throw new Exception("The updater threw an error.");
                }
            }
#else

            Updater.setCallback(UpdaterProgress);
            await Task.Run(() =>
            {
                try
                {
                    if (DeviceModel.IsSerial)
                        IsConnectedTimer.Stop();

                    if(InSecurity)
                    {
                        if (UnlockWithBackdoorKey == true)
                            Updater.unlockWithKey(BackdoorKey);
                        else if (FlashEraseAllUnsecure == true)
                            Updater.eraseAllUnsecure();
                        Status = "Unlocked";

                    }
                    Updater.flashFirmware(ImageFileModel.FullPath, ImageFileModel.BaseAddress);

                    Reset();
                    if (DeviceModel.IsSerial)
                        Status = "Complete! Waiting for application to start.";
                    else
                        Status = "Complete! Press Reset button to start application.";
                    Value = 100;

                    if (DeviceModel.IsSerial)
                    {
                        IsConnectedTimer.Interval = new TimeSpan(0, 0, 10);
                        IsConnectedTimer.Start();
                    }
                }
                catch (Exception e)
                { 
                    if (DeviceModel.IsSerial)
                        IsConnectedTimer.Start();
                    throw e;
                }
            }, token);
#endif
        } // Update()

        //Update the UI to reflect the progress value that is passed back.
        private void UpdaterProgress(blfwkdll.Updater.UpdaterOperationProgressData opData)
        {
//            Value = (Int32)opData.Tasks[opData.CurrentTaskIndex].CurrentPosition;
//            LogTextBox.AppendText(String.Format("Progress: {0} Current: {1}\r\n", opData.Tasks[opData.CurrentTaskIndex].TaskDescription.ToString(), opData.Tasks[opData.CurrentTaskIndex].CurrentPosition));
            Status = opData.Tasks[opData.CurrentTaskIndex].TaskDescription.Description;
            Value = ((opData.CurrentTaskIndex) * 100) / opData.Tasks.Count;
//            return !_UserStopped;

        }

    } // class UpdaterViewModel
}

