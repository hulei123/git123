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
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using DevSupport.DeviceManager;
using KinetisUpdater.ViewModel;

namespace KinetisUpdater
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
/*        public ObservableCollection<ImageFileViewModel> ImageFileViewModelList { get; private set; }
        private ImageFileViewModel _CurrentImageFile = new ImageFileViewModel();
        public ImageFileViewModel CurrentImageFile
        {
            get { return _CurrentImageFile; }
            set
            {
                if (_CurrentImageFile != value)
                {
                    _CurrentImageFile = value;
                    if (!_CurrentImageFile.IsNull)
                    {
                        Properties.Settings.Default.LastDeviceInstance = _CurrentImageFile.DeviceInstance;
                        _CurrentImageFile.BootloaderModel.Ping();
                    }
                    RaisePropertyChangedEvent("CurrentImageFile");
                }
            }
        }
*/
        public MainWindow()
        {
            InitializeComponent();
            
            // The following snipped shows how to adjust the trace level by code:
            // PresentationTraceSources.DataBindingSource.Listeners.Add(new ConsoleTraceListener());
            // PresentationTraceSources.DataBindingSource.Switch.Level = SourceLevels.All;
            
            _mainFrame.Navigate(new HomePage((DeviceManagerViewModel)this.DataContext));
        }

        // Restores the main window size and position from last session.
        private void MainWindow_Activated(object sender, EventArgs e)
        {
            if (!_FirstActivation)
                return;

            _FirstActivation = false;

            try
            {
                Rect bounds = Properties.Settings.Default.WindowPosition;
                if (bounds != new Rect(0, 0, 0, 0))
                {
                    this.Top = bounds.Top;
                    this.Left = bounds.Left;
                    this.Width = bounds.Width;
                    this.Height = bounds.Height;
                }
            }
            catch { }
        }
        private bool _FirstActivation = true;

        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            Properties.Settings.Default.WindowPosition = this.RestoreBounds;
            Properties.Settings.Default.Save();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // Add "About..." menu item to system menu.
//            IntPtr sysMenuHandle = NativeMethods.GetSystemMenu(new WindowInteropHelper(this).Handle, false);
//            NativeMethods.AppendMenu(sysMenuHandle, NativeMethods.MF_SEPARATOR, 0, string.Empty);
//            NativeMethods.AppendMenu(sysMenuHandle, NativeMethods.MF_STRING, NativeMethods.IDM_ABOUT, "About " + this.Title + "...");


//            DeviceManagerViewModel.SetCurrentDevice(Properties.Settings.Default.DeviceInstance);
//            DeviceInfo.DataContext = DeviceManagerViewModel.CurrentDevice;

//            FileHistory = ImageFileModel.ToImageModels(Properties.Settings.Default.FileHistory);
//            ImageFileModel = FileHistory.FirstOrDefault(i => i.FullPath == Properties.Settings.Default.LastFilename);
//            if (ImageFileModel != null)
//            {
//                ImageFileModel.BaseAddress = Properties.Settings.Default.BaseAddress;
//                LogTextBox.AppendText(String.Format("User set Image.BaseAddress to 0x{0:X8}.\r\n", ImageFileModel.BaseAddress));
//            }

//            UpdateStatus(UpdaterModel.Transition.ImageChange);
        }





    }
}
