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
//using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
//using System.ComponentModel;
//using System.Diagnostics;
using System.IO;
using System.Linq;
//using System.Threading;
//using System.Windows.Data;
using System.Windows.Input;
//using System.Windows.Media.Imaging;
//using System.Windows.Threading;

using Microsoft.Win32;
//using DevSupport.DeviceManager;

namespace KinetisUpdater.ViewModel
{
    //==============================================================================
    //==============================================================================
    /// <summary>
    /// ViewModel for ImageFileManager
    /// </summary>
    public class ImageFileManagerViewModel : ObservableObject
    {
        public ObservableCollection<ImageFileViewModel> ImageFileViewModelList { get; private set; }
        private ImageFileViewModel _CurrentImageFile = new ImageFileViewModel();
        public ImageFileViewModel CurrentImageFile
        {
            get { return _CurrentImageFile; }
            set
            {
                if (_CurrentImageFile != value)
                {
                    if (value == null)
                        _CurrentImageFile = new ImageFileViewModel();
                    else
                        _CurrentImageFile = value;

                    if (!_CurrentImageFile.IsNull)
                    {
                        Properties.Settings.Default.LastFilename = _CurrentImageFile.FullPath;
                    }
                    RaisePropertyChangedEvent("CurrentImageFile");
                }
            }
        }

        public ImageFileManagerViewModel()
        {
            ImageFileViewModelList = ToImageFileViewModels(Properties.Settings.Default.FileHistory);
            CurrentImageFile = ImageFileViewModelList.FirstOrDefault(i => i.FullPath == Properties.Settings.Default.LastFilename);
            if (!CurrentImageFile.IsNull)
            {
                CurrentImageFile.BaseAddress = Properties.Settings.Default.BaseAddress;
            }
        }

        public static StringCollection ToStringCollection(ObservableCollection<ImageFileViewModel> models)
        {
            var collection = new StringCollection();
            if (models != null)
            {
                foreach (var model in models)
                    collection.Add(model.FullPath);
            }
            return collection;
        }

        public static ObservableCollection<ImageFileViewModel> ToImageFileViewModels(StringCollection paths)
        {
            var models = new ObservableCollection<ImageFileViewModel>();
            foreach (var path in paths)
            {
                if ( File.Exists(path) )
                    models.Add(new ImageFileViewModel(path));
            }

            return models;
        }

        public ICommand BrowseForImageFileCommand
        {
            get { return new DelegateCommand(BrowseForImageFile); }
        }

        private void BrowseForImageFile()
        {
            OpenFileDialog openDlg = new OpenFileDialog();
            //openDlg.Filter = "Kinetis Update image files (*.bin, *.srec, *.sb, *.hex, *.elf)|*.bin;*.srec;*.sb;*.hex;*.elf|All files(*.*)|*.*";
            openDlg.Filter = "Kinetis Update image files (*.bin, *.sb)|*.bin;*.sb|All files(*.*)|*.*";
            openDlg.CheckFileExists = true;

            if (openDlg.ShowDialog() == true)
            {
                ImageFileViewModelList.Remove(ImageFileViewModelList.FirstOrDefault(i => i.FullPath == openDlg.FileName));
                ImageFileViewModelList.Insert(0, new ImageFileViewModel(openDlg.FileName));
                CurrentImageFile = ImageFileViewModelList[0];
                while (ImageFileViewModelList.Count > 10)
                    ImageFileViewModelList.RemoveAt(10);
                Properties.Settings.Default.FileHistory = ToStringCollection(ImageFileViewModelList);
            }

        } // BrowseForImageFile()

        public ICommand CheckAddressInput
        {
            get
            {
                return new DelegateCommand<KeyEventArgs>((e) =>
                {
                    e.Handled = KeyInputFilter.Filter(e.Key, e.KeyboardDevice.Modifiers);
                }
                );
            }
        }
    } // class ImageFileManagerViewModel


//==============================================================================
//==============================================================================
    /// <summary>
    /// ViewModel for ImageFile
    /// </summary>
    public class ImageFileViewModel : ObservableObject
    {
        public String Filename { get { return Path.GetFileName(FullPath); } }
        public String FullPath { get; private set; }
        public UInt32 BaseAddress
        {
            get { return _BaseAddress; }
 
            set
            {
                if (_BaseAddress != value)
                {
                    _BaseAddress = value;
                    Properties.Settings.Default.BaseAddress = _BaseAddress;
                    RaisePropertyChangedEvent("BaseAddress");
                }
            }
        }
        private UInt32 _BaseAddress = 0;

        public ImageFileViewModel()
        {
            FullPath = null;
        }

        public ImageFileViewModel(String fullFilename)
        {
            FullPath = fullFilename;
            BaseAddress = Properties.Settings.Default.BaseAddress;
        }

        public bool IsNull { get { return String.IsNullOrEmpty(this.FullPath); } }

        public bool IsEnabled { get { return !IsNull; } }

        public override string ToString()
        {
            return IsNull ? "Select an image file" : this.Filename;
        }

    }

}

