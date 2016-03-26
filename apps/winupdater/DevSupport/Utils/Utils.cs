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
using System.ComponentModel.Design;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using System.ComponentModel;
using System.Reflection;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;

namespace Utils
{
    #region Template Selectors

    public class ComboBoxItemTemplateSelector : DataTemplateSelector
    {
        public DataTemplate SelectedTemplate { get; set; }
        public DataTemplate DropDownTemplate { get; set; }

        public override DataTemplate SelectTemplate(object item, DependencyObject container)
        {
            ComboBoxItem comboBoxItem = container.GetVisualParent<ComboBoxItem>();
            if (comboBoxItem != null)
            {
                return DropDownTemplate;
            }
            return SelectedTemplate;
        }
    }
    
    #endregion

    #region Type Converters

    public class DecimalConverterEx : DecimalConverter
    {
//        public DecimalConverterEx(Type type) { ConvertToType = type; }
//        private Type ConvertToType;

        public override object ConvertFrom(ITypeDescriptorContext context, System.Globalization.CultureInfo culture, object value)
        {
            string s = value as string;
            if (s != null)
            {
                System.Globalization.NumberStyles numStyle;
                String[] sArr = s.Split(' ');
                if (sArr.Length > 1)
                    s = sArr[0];

                if (s.StartsWith("0x"))
                {
                    s = s.Substring(2);
                    numStyle = System.Globalization.NumberStyles.HexNumber;
                }
                else
                {
                    s = s.Replace("(", ""); s = s.Replace(")", "");
                    numStyle = System.Globalization.NumberStyles.Integer | System.Globalization.NumberStyles.AllowThousands;
                }

                if (context.PropertyDescriptor.PropertyType == typeof(Byte))
                    return Byte.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(SByte))
                    return SByte.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(UInt16))
                    return UInt16.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(Int16))
                    return Int16.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(UInt32))
                    return UInt32.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(Int32))
                    return Int32.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(UInt64))
                    return UInt64.Parse(s, numStyle);
                else if (context.PropertyDescriptor.PropertyType == typeof(Int64))
                    return Int64.Parse(s, numStyle);
                else
                    throw new NotSupportedException();
            }
            return base.ConvertFrom(context, culture, value);
        }

        public override object ConvertTo(ITypeDescriptorContext context, System.Globalization.CultureInfo culture, object value, Type destinationType)
        {
            String formatString = "0x{0:X" + Convert.ToString(Marshal.SizeOf(value) * 2) + "} ({1})";

            UInt64 uint64;
            if (UInt64.TryParse(value.ToString(), out uint64))
            {
                return destinationType == typeof(string)
                    ? String.Format(formatString, uint64, uint64.ToString("#,#0"))
                    : base.ConvertTo(context, culture, value, destinationType);
            }
            else
            {
                return destinationType == typeof(string)
                    ? String.Format(formatString, value, Convert.ToInt64(value).ToString("#,#0"))
                    : base.ConvertTo(context, culture, value, destinationType);
            }
        }
    }

    public class ByteFormatConverter : DecimalConverter
    {
        public override object ConvertFrom(ITypeDescriptorContext context, System.Globalization.CultureInfo culture, object value)
        {
            string s = value as string;
            if (s != null)
            {
                String[] sArr = s.Split(' ');
                Double size = 0;

                if (sArr.Length > 1)
                {
                    if (sArr[1] == "GB")
                        size = Convert.ToDouble(sArr[0]) * 1024 * 1024 * 1024;
                    else if (sArr[1] == "MB")
                        size = Convert.ToDouble(sArr[0]) * 1024 * 1024;
                    else if (sArr[1] == "KB")
                        size = Convert.ToDouble(sArr[0]) * 1024;
                    else
                        size = Convert.ToDouble(sArr[0]);

                    return Convert.ToInt64(size);
                }
                else if (sArr.Length == 1)
                {
                    return Convert.ToInt64(sArr[0]);
                }
            }
            return base.ConvertFrom(context, culture, value);
        }
        public override object ConvertTo(ITypeDescriptorContext context, System.Globalization.CultureInfo culture, object value, Type destinationType)
        {
            return destinationType == typeof(string)
                ? Utils.ScaleBytes(value) + String.Format(" ({0} bytes)", Convert.ToUInt64(value).ToString("#,#0"))
                : base.ConvertTo(context, culture, value, destinationType);
        }
    }

    public class EnumConverterEx : EnumConverter
    {
        public EnumConverterEx(Type enumType) : base(enumType) { }

        public override object ConvertFrom(ITypeDescriptorContext context, System.Globalization.CultureInfo culture, object value)
        {
            string s = value as string;
            if (s != null)
            {
                String[] sArr = s.Split(' ');
                return Enum.Parse(EnumType, sArr[0]);
            }
            return base.ConvertFrom(context, culture, value);
        }
        public override object ConvertTo(ITypeDescriptorContext context, System.Globalization.CultureInfo culture, object value, Type destinationType)
        {
            return destinationType == typeof(string)
                ? String.Format("{0} (0x{0:X})", value)
                : base.ConvertTo(context, culture, value, destinationType);
        }
    }

    #endregion

    #region Value Converters

    [ValueConversion(typeof(UInt32), typeof(String))]
    public class DecToHexConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return String.Format("0x{0:X8}",value);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            UInt32 outputUInt32 = 0;
            var inputString = value as String;
            if (inputString.StartsWith("0x"))
                UInt32.TryParse(inputString.Substring(2), NumberStyles.HexNumber, null, out outputUInt32);
            else
                UInt32.TryParse(inputString, NumberStyles.Any, null, out outputUInt32);

            return outputUInt32;
        }
    }

    [ValueConversion(typeof(String), typeof(Boolean))]
    public class AccessToEnabledConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            Boolean enabled = true;

            switch ((String)value)
            {
                case "RO":
                    enabled = false;
                    break;
                case "ROZ":
                    enabled = false;
                    break;
            }
            return enabled;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return (Boolean)value ? "RW" : "RO";
        }
    }

    [ValueConversion(typeof(String), typeof(String))]
    public class FieldDescriptionConverter : IMultiValueConverter
    {
        public object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
        {
            String desc = values[0].ToString();
            String note = null;
            if ( values[1] != null )
                note = (String)values[1].ToString();

            return String.Format("{0}{1}", desc, String.IsNullOrEmpty(note) ? String.Empty : String.Format("\r\n\r\nNOTE: {0}", note));
        }

        public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
        {
            string[] delimeters = new string[] { "\r\nNOTE: " };
            string[] splitValues = ((string)value).Split(delimeters,  StringSplitOptions.RemoveEmptyEntries);
            return splitValues;
        }
    }

    [ValueConversion(typeof(String), typeof(String))]
    public class FilenameConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return Path.GetFileName(value.ToString());
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return value;
        }
    }

    // Invertable BooleanToVisibilityConverter
    public class BooleanToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            //  Cast the data to a bool.
            bool booleanValue;
            try
            {
                booleanValue = (bool)value;
            }
            catch (Exception exception)
            {
                throw new InvalidOperationException("The value provided to a BooleanToVisibilityConverter could not be cast to a boolean.", exception);
            }

            //  Are we inverting?
            bool invert = parameter != null && parameter.ToString().Contains("Invert");
            bool hidden = parameter != null && parameter.ToString().Contains("Hidden");

            //  Return the appropriate visibility.
            if (invert)
            {
                if (hidden)
                    return booleanValue ? Visibility.Hidden : Visibility.Visible;
                else
                    return booleanValue ? Visibility.Collapsed : Visibility.Visible;
            }
            else
            {
                if (hidden)
                    return booleanValue ? Visibility.Visible : Visibility.Hidden;
                else
                    return booleanValue ? Visibility.Visible : Visibility.Collapsed;
            }
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            //  Cast the data to a bool.
            Visibility visibilityValue;
            try
            {
                visibilityValue = (Visibility)value;
            }
            catch (Exception exception)
            {
                throw new InvalidOperationException("The value provided to a BooleanToVisibilityConverter could not be cast to a Visibility.", exception);
            }

            //  Are we inverting?
            bool invert = parameter != null &&
                   String.Compare(parameter.ToString(), "Invert", StringComparison.InvariantCultureIgnoreCase) == 0;

            //  Return the appropriate visibility.
            if (invert)
                return visibilityValue == Visibility.Visible ? false : true;
            else
                return visibilityValue == Visibility.Visible ? true : false;
        }
    }

    // This converter performs a logical AND of all parameters. 
    public class MultiBoolConverter : IMultiValueConverter
    {
        public virtual object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
        {
            if (values.Length == 0)
                return false;

            bool retVal = true;

            foreach ( var val in values )
            {
                bool? boolValue = val as Boolean?;
                retVal &= boolValue == null ? false : (bool)boolValue;
                
            }
            return retVal;
        }

        public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// This converter does nothing except breaking the
    /// debugger into the convert method
    /// </summary>
    public class DatabindingDebugConverter : IValueConverter
    {
        public object Convert(object value, Type targetType,
            object parameter, CultureInfo culture)
        {
            Debugger.Break();
            return value;
        }

        public object ConvertBack(object value, Type targetType,
            object parameter, CultureInfo culture)
        {
            Debugger.Break();
            return value;
        }
    }

    /// <summary>
    /// This converter does nothing except breaking the
    /// debugger into the convert method
    /// </summary>
    public class DatabindingDebugMultiConverter : IMultiValueConverter
    {
        public virtual object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
        {
            Debugger.Break();
            return values[0];
        }

        public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
        {
            Debugger.Break();
            return new object[2] {value, value};
        }
    }
    #endregion

    #region Validation Rules

    public class FileExistsValidationRule : ValidationRule
    {
        public String ErrorMessage { get; set; }

        public override ValidationResult Validate(object value,
            CultureInfo cultureInfo)
        {
            ValidationResult result = new ValidationResult(true, null);
            string inputString = (value ?? string.Empty).ToString();
            if (!File.Exists(inputString))
            {
                ErrorMessage = String.Format("Cannot find file \"{0}\".", inputString);
                result = new ValidationResult(false, this.ErrorMessage);
            }
            return result;
        }
    }

    #endregion

    static public class Utils
    {
        public static String StringFromAsciiBytes(Byte[] bytes)
        {
            System.Text.ASCIIEncoding encoder = new System.Text.ASCIIEncoding();
            return encoder.GetString(bytes);
        }

        public static String StringFromAsciiBytes(Byte[] bytes, int byteIndex, int byteCount)
        {
            System.Text.ASCIIEncoding encoder = new System.Text.ASCIIEncoding();
            return encoder.GetString(bytes, byteIndex, byteCount);
        }

        public static Byte[] StringToAsciiBytes(String inputString)
        {
            System.Text.ASCIIEncoding  encoder = new System.Text.ASCIIEncoding();
            return encoder.GetBytes(inputString);
        }

        public static String StringFromUnicodeBytes(Byte[] bytes)
        {
            System.Text.UnicodeEncoding encoder = new System.Text.UnicodeEncoding();
            return encoder.GetString(bytes);
        }

        public static String StringFromHexBytes(Byte[] bytes, String spaces)
        {
            String responseStr = String.Empty;

            foreach (Byte dataByte in bytes )
            {
                responseStr += String.Format("{0:X2}{1}", dataByte, spaces);
            }
            return responseStr;
        }

        // GB, MB, KB, bytes
        public static String ScaleBytes(object sizeInBytes)
        {
            double originalSize = Convert.ToDouble(sizeInBytes);
            double scaledSize = 0;

            scaledSize = originalSize / (1024 * 1024 * 1024);
            if (scaledSize > 1.0) // GB
            {
                return scaledSize.ToString("#,#.###' GB'");
            }
            
            scaledSize = originalSize / (1024 * 1024);
            if (scaledSize > 1.0) // MB
            {
                return scaledSize.ToString("#.###' MB'");
            }

            scaledSize = originalSize / 1024;
            if (scaledSize > 1.0) // KB
            {
                return scaledSize.ToString("#.###' KB'");
            }
            else // bytes
            {
                return originalSize.ToString("#,0' bytes'");
            }

        } // ScaleBytes()

        public static object ByteArrayToStructure(byte[] bytes, Type structureType)
        {

            GCHandle hDataIn = GCHandle.Alloc(bytes, GCHandleType.Pinned);

            object retObject = Marshal.PtrToStructure(hDataIn.AddrOfPinnedObject(), structureType);
            
            hDataIn.Free();

            return retObject;
        }

        ////////////////////////////////////////////////////////////////////////////////
        //! Converts a four digit BCD number to the decimal equivalent.
        //! Remember that the BCD value is big endian but read as a 16-bit little 
        //! endian number. So we have to byte swap during this conversion.
        //!
        //! \param bcdNumber BCD value in reverse byte order.
        //! \return A decimal version of \a bcdNumber.
        ////////////////////////////////////////////////////////////////////////////////
        public static UInt16 BcdToDecimal(UInt16 bcdNumber)
        {
            UInt16 resultVersion = 0;
            resultVersion = (UInt16)((bcdNumber & 0x0000000f) * 100);
            resultVersion += (UInt16)(((bcdNumber & 0x000000f0) >> 4) * 1000);
            resultVersion += (UInt16)(((bcdNumber & 0x00000f00) >> 8) * 1);
            resultVersion += (UInt16)(((bcdNumber & 0x0000f000) >> 12) * 10);
            return resultVersion;
        }


        public static T GetVisualParent<T>(this DependencyObject child) where T : Visual
        {
            while ((child != null) && !(child is T))
            {
                child = VisualTreeHelper.GetParent(child);
            }
            return child as T;
        }

    } // class Utils
}
