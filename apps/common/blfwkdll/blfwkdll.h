/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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
// blfwkdll.h

#pragma once

using namespace System;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;

#include "blfwk/Bootloader.h"
#include "blfwk/Updater.h"
//#include "blfwk/host_peripheral.h"
#include "blfwk/SerialPacketizer.h"
//#include "bootloader_common.h"

//#pragma make_public(blfwk::Peripheral)
//#pragma make_public(blfwk::Bootloader)
//#pragma make_public(blfwk::Updater)

namespace blfwkdll
{
    // From property/property.h
    //! @brief Structure of version property.
    [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi)]
    public value struct StandardVersion
    {
    public:
        //! @brief bugfix version [7:0]
        property Byte bugfix
        {
            Byte get() { return BitConverter::GetBytes(_Version)[0]; }
        }

        //! @brief minor version [15:8]
        property Byte minor
        {
            Byte get() { return BitConverter::GetBytes(_Version)[1]; }
        }

        //! @brief major version [23:16]
        property Byte major
        {
            Byte get() { return BitConverter::GetBytes(_Version)[2]; }
        }

        //! @brief name [31:24]
        property Byte name
        {
            Byte get() { return BitConverter::GetBytes(_Version)[3]; }
        }

        //! @brief combined version numbers
        property UInt32 version
        {
            UInt32 get() { return _Version; }
        }

        StandardVersion(standard_version_t version)
            : _Version((UInt32)version.version)
        {}

        virtual String ^ ToString() override
        {
            return String::Format("{0}{1}.{2}.{3}", Convert::ToChar(name), major, minor, bugfix);
        }

        private:
            UInt32 _Version;

    };    // struct StandardVersion

    //! @brief Structure of target security state.
    [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi)]
    public value struct SecurityState
    {
    public:
        //! @brief target flash security state
        property UInt32 state
        {
            UInt32 get() { return _State; }
        }

        SecurityState(UInt32 state)
            : _State(state)
        {}
        //! Convert the flash security state to String.
        virtual String ^ ToString() override
        {
            switch (state)
            {
            case 0: //  kFlashNotSecure
                return String::Format("UnSecure");
            case 1: //  kFlashSecureBackdoorEnabled
                //return String::Format("SecureWithBackdoorKey");
            case 2: //  kFlashSecureBackdoorDisabled
                //return String::Format("SecureWithoutBackdoorKey");
                return String::Format("Secure");
            default:
                return String::Format("UnKnown");
            }
        }
        //! Convert the flash security state to Boolean.
        virtual Boolean ToBoolean()
        {
            return state ? true : false;
        }

    private:
        UInt32 _State;

    };    // struct SecurityState

    // From blfwk/bus_pal.h
    public ref class BusPal
    {
    public:
        //! @brief BusPal Transports.
        enum class BusPalTransports : Int32
        {
            None = blfwk::BusPal::kBusPalFunction_None,
            SPI  = blfwk::BusPal::kBusPalFunction_SPI,
            I2C  = blfwk::BusPal::kBusPalFunction_I2C
        };

        //! @brief SPI clock polarity configuration.
        enum class SpiClockPolarities : Int32
        {
            ActiveHigh = blfwk::BusPal::kSpiClockPolarity_ActiveHigh,   //!< Active-high SPI clock (idles low).
            ActiveLow  = blfwk::BusPal::kSpiClockPolarity_ActiveLow     //!< Active-low SPI clock (idles high).
        };

        //! @brief SPI clock phase configuration.
        enum class SpiClockPhases : Int32
        {
            FirstEdge  = blfwk::BusPal::kSpiClockPhase_FirstEdge,       //!< First edge on SPSCK occurs at the middle of the first cycle of a data transfer.
            SecondEdge = blfwk::BusPal::kSpiClockPhase_SecondEdge       //!< First edge on SPSCK occurs at the start of the first cycle of a data transfer.
        };

        //! @brief SPI data shifter direction options.
        enum class SpiShiftDirections : Int32
        {
            MsbFirst = blfwk::BusPal::kSpiMsbFirst,    //!< Data transfers start with most significant bit.
            LsbFirst = blfwk::BusPal::kSpiLsbFirst     //!< Data transfers start with least significant bit.
        };

        //! @brief BusPal configuration data.
        ref class BusPalConfigData
        {
        public:
            property BusPalTransports transport
            {
                BusPalTransports get() { return (BusPalTransports)m_ptr->function; }
                void set(BusPalTransports value) { m_ptr->function = (blfwk::BusPal::bus_pal_function_t)value; }
            }
            property UInt32 spiSpeedKHz
            {
                UInt32 get() { return m_ptr->spiSpeedKHz; }
                void set(UInt32 value) { m_ptr->spiSpeedKHz = value; }
            }
            property SpiClockPolarities spiPolarity
            {
                SpiClockPolarities get() { return (SpiClockPolarities)m_ptr->spiPolarity; }
                void set(SpiClockPolarities value) { m_ptr->spiPolarity = (blfwk::BusPal::spi_clock_polarity_t)value; }
            }
            property SpiClockPhases spiPhase
            {
                SpiClockPhases get() { return (SpiClockPhases)m_ptr->spiPhase; }
                void set(SpiClockPhases value) { m_ptr->spiPhase = (blfwk::BusPal::spi_clock_phase_t)value; }
            }
            property SpiShiftDirections spiDirection
            {
                SpiShiftDirections get() { return (SpiShiftDirections)m_ptr->spiDirection; }
                void set(SpiShiftDirections value) { m_ptr->spiDirection = (blfwk::BusPal::spi_shift_direction_t)value; }
            }
            property Byte i2cAddress
            {
                Byte get() { return m_ptr->i2cAddress; }
                void set(Byte value) { m_ptr->i2cAddress = value; }
            }
            property UInt32 i2cSpeedKHz
            {
                UInt32 get() { return m_ptr->i2cSpeedKHz; }
                void set(UInt32 value) { m_ptr->i2cSpeedKHz = value; }
            }

            BusPalConfigData(blfwk::BusPal::BusPalConfigData * ptr)
                : m_ptr(ptr)
            {}

        private:
            blfwk::BusPal::BusPalConfigData * m_ptr;

        };    // struct BusPalConfigData

    };        // class BusPal

    // From blfwk/host_peripheral.h
    public ref class Peripheral
    {
    public:
        enum class HostPeripheralTypes : Int32
        {
            None        = blfwk::Peripheral::kHostPeripheralType_None,
            UART        = blfwk::Peripheral::kHostPeripheralType_UART,
            BusPal_UART = blfwk::Peripheral::kHostPeripheralType_BUSPAL_UART,
            USB_HID     = blfwk::Peripheral::kHostPeripheralType_USB_HID
        };

        [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi)]
        ref class PeripheralConfigData
        {
        public:
            property HostPeripheralTypes peripheralType
            {
                HostPeripheralTypes get()
                {
                    return static_cast<HostPeripheralTypes>(m_ptr->peripheralType);
                }

                void set(HostPeripheralTypes value)
                {
                    m_ptr->peripheralType = static_cast<blfwk::Peripheral::_host_peripheral_types>(value);
                }
            }

            [MarshalAs(UnmanagedType::LPStr)]
            property String^ comPortName
            {
                String^ get()
                {
                    return gcnew String(m_ptr->comPortName.c_str());
                }

                void set(String^ value)
                {
                    IntPtr ip = Marshal::StringToHGlobalAnsi(value);
                    m_ptr->comPortName = static_cast<const char*>(ip.ToPointer());
                    Marshal::FreeHGlobal(ip);
                }
            }

            property UInt32 comPortSpeed
            {
                UInt32 get() { return m_ptr->comPortSpeed; }
                void set(UInt32 value) { m_ptr->comPortSpeed = value; }
            }

            property UInt32 serialReadTimeoutMs
            {
                UInt32 get() { return m_ptr->packetTimeoutMs; }
                void set(UInt32 value) { m_ptr->packetTimeoutMs = value; }
            }

            property UInt16 usbHidVid
            {
                UInt16 get() { return m_ptr->usbHidVid; }
                void set(UInt16 value) { m_ptr->usbHidVid = value; }
            }

            property UInt16 usbHidPid
            {
                UInt16 get() { return m_ptr->usbHidPid; }
                void set(UInt16 value) { m_ptr->usbHidPid = value; }
            }

            [MarshalAsAttribute(UnmanagedType::LPStr)]
            property String^ usbHidSerialNumber
            {
                String^ get()
                {
                    return gcnew String(m_ptr->usbHidSerialNumber.c_str());

                }

                void set(String^ value)
                {
                    IntPtr ip = Marshal::StringToHGlobalAnsi(value);
                    m_ptr->comPortName = static_cast<const char*>(ip.ToPointer());
                    Marshal::FreeHGlobal(ip);
                }
            }

            [MarshalAsAttribute(UnmanagedType::I1)]
            property Boolean usePing
            {
                Boolean get() { return m_ptr->ping; }
                void set(Boolean value) { m_ptr->ping = value; }
            }

            property BusPal::BusPalConfigData^ busPalConfig;

            PeripheralConfigData()
                : m_ptr(new blfwk::Peripheral::PeripheralConfigData())
            {
                busPalConfig = gcnew BusPal::BusPalConfigData(&m_ptr->busPalConfig);
            }

            PeripheralConfigData(HostPeripheralTypes type)
                : m_ptr(new blfwk::Peripheral::PeripheralConfigData())
            {
                peripheralType = type;
                busPalConfig = gcnew BusPal::BusPalConfigData(&m_ptr->busPalConfig);
            }

            ~PeripheralConfigData() { this->!PeripheralConfigData(); }
            !PeripheralConfigData() { delete m_ptr; }

            property blfwk::Peripheral::PeripheralConfigData & Native
            {
                blfwk::Peripheral::PeripheralConfigData & get() { return *m_ptr; }
            }

        private:
            blfwk::Peripheral::PeripheralConfigData * m_ptr;

        };  // class PeripheralConfigData
    };      // class Peripheral

/*
    // From blfwk/host_bootloader.h.h
    public ref class Bootloader : public IDisposable
    {
    public:
        //! @brief Initialize the singleton Bootloader and return a reference.
        //!
        //! Populate the the peripheralType member and any other
        //! members relevant to the stated peripheralType.
        //!
        //! @param params Peripheral configuration parameter structure.
        //! @retval Peripheral object
        Bootloader(Peripheral::PeripheralConfigData^ config)
            : m_disposed(false)
        {
            m_nativeBootloader = NULL;
            pin_ptr<blfwk::Peripheral::PeripheralConfigData> pinnedConfig = &(config->Native);

            try
            {
                 m_nativeBootloader = new blfwk::Bootloader(*pinnedConfig);
            }
            catch (exception e)
            {
                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Bootloader() rethrowing: {0}"), gcnew String(e.what())));
            }
        }

        property blfwk::Bootloader * Native
        {
            blfwk::Bootloader * get() { return m_nativeBootloader; }
        }

        // This method is called if the user explicitly disposes of the
        // object by calling the C++ destructor. The compiler emits as a call to 
        // GC::SuppressFinalize( this ) for you, so there is no need to 
        // call it here.
        ~Bootloader()
        {
            if ( !m_disposed && m_nativeBootloader != nullptr)
            {
                delete m_nativeBootloader;
                m_nativeBootloader = nullptr;
                m_disposed = true;
            }
        }
        
        // The C++ finalizer destructor ensures that unmanaged resources get
        // released if the user releases the object without explicitly 
        // disposing of it.
        //
        !Bootloader()
        {
            this->~Bootloader();
        }

        property StandardVersion SerialProtocolVersion
        {
            StandardVersion get()
            {
                blfwk::SerialPacketizer * packetizer = dynamic_cast<blfwk::SerialPacketizer *>(m_nativeBootloader->getPacketizer());
                if ( packetizer )
                    return StandardVersion(packetizer->getVersion());

                return StandardVersion();
            }
        }

        property StandardVersion Version
        {
            StandardVersion get() { return StandardVersion(getVersion()); }
        }

//        void flush()
//        {
//            m_nativeBootloader->flush();
//        }

//        void reset()
//        {
//            try
//            {
//                m_nativeBootloader->reset();
//            }
//            catch (exception e)
//            {
//                delete this;
//                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Bootloader.reset() rethrowing: {0}"), gcnew String(e.what())));
//            }
//        }

        standard_version_t getVersion()
        {
            try
            {
                 return m_nativeBootloader->getVersion();
            }
            catch (exception e)
            {
                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Bootloader.getVersion() rethrowing: {0}"), gcnew String(e.what())));
            }
        }

        void ping(Int32 retries, UInt32 delay, Int32 comSpeed)
        {
            try
            {
                 m_nativeBootloader->ping(retries, delay, comSpeed);
            }
            catch (exception e)
            {
                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Bootloader.ping() rethrowing: {0}"), gcnew String(e.what())));
            }
        }

    private:
        blfwk::Bootloader * m_nativeBootloader; 

        /// <summary>
        /// Used to determine if Dispose() has already been called.
        /// </summary>
        bool m_disposed;
    };
*/
    // From blfwk/Updater.h
    public ref class Updater 
    {
    public:
        //! @name Updater states with descriptions.
        //@{
        [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi)]
        ref class UpdaterEnum
        { 
        public:
            UInt32 Value;
            String^ Description;

            virtual String^ ToString() override { return String::Format("{0} ({1})", Description, Value); } 

            UpdaterEnum(blfwk::updater_enum_t updaterEnum)
            {
                Description = gcnew String(updaterEnum.description.c_str());
                Value = updaterEnum.value;
            }

            UpdaterEnum(UInt32 value, String^ description)
            {
                Description = description;
                Value = value;
            }
        };
        //@}

        //! @name Updater tasks with descriptions.
        //@{
        [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi)]
        ref class UpdaterTask
        {
        public:
            UpdaterEnum^ TaskDescription;
            UInt32 CurrentPosition;
            UInt32 Total;

            UpdaterTask(blfwk::updater_task_t updaterTask)
            {
                TaskDescription = gcnew UpdaterEnum(updaterTask.task_desc);
                CurrentPosition = updaterTask.current;
                Total = updaterTask.total;
            }

            UpdaterTask(UpdaterEnum^ description, UInt32 current, UInt32 total)
            {
                TaskDescription = description;
                CurrentPosition = current;
                Total = total;
            }
        };

        //! @name Updater operation.
        //@{
        [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi)]
        ref class UpdaterOperationProgressData
        {
        public:
            UpdaterEnum^ OperationDesc;
            List<UpdaterTask^>^ Tasks;
            Int32 CurrentTaskIndex;
            UInt32 CurrentPosition;
            UInt32 Total;

            UpdaterOperationProgressData(blfwk::updater_operation_t * op)
            {
                OperationDesc = gcnew UpdaterEnum(op->operation_desc);
                Tasks = gcnew List<UpdaterTask^>();
                for each (blfwk::updater_task_t task in op->tasks)
                {
                    Tasks->Add(gcnew UpdaterTask(task));
                }
                CurrentTaskIndex = op->current_task;
                CurrentPosition = op->current();
                Total = op->total();
            }

            UpdaterOperationProgressData(UpdaterEnum^ description)
            {
                OperationDesc = description;
                Tasks = gcnew List<UpdaterTask^>();
                CurrentTaskIndex = 0;
                CurrentPosition = 0;
                Total = 0;
            }
        };
        //@}

        //! @brief Type for the progress callback routine used from managed code.
        delegate void UpdaterOperationProgressDelegate(Updater::UpdaterOperationProgressData^ opData);

    private:
        //! @brief The intermediate managed delegate definition.
        [UnmanagedFunctionPointer(CallingConvention::Cdecl)]
        delegate void IntermediateUpdaterCallbackDelegate(blfwk::updater_operation_t * op);

        //! @brief The instance of the intermediate managed delgate.
        IntermediateUpdaterCallbackDelegate^ _IntermediateCallback;

        ///! @brief The managed delegate ref to be used later
        UpdaterOperationProgressDelegate ^ _ManagedWrapperCallback;

        //! @brief Hold a handle to the callback passed to the unmanaged code so it doesn't get 
        // garbage collected while it is being used. Don't forget to release it when we are done.
        GCHandle _IntermediateCallbackHandle;
        //! @brief Underlying unmanaged object.
        blfwk::Updater * _UnmanagedUpdater;

    public:
        Updater(Peripheral::PeripheralConfigData^ config)
        {
            pin_ptr<blfwk::Peripheral::PeripheralConfigData> pinnedConfig = &(config->Native);
            _UnmanagedUpdater = nullptr;
            try
            {
                _UnmanagedUpdater = new blfwk::Updater(*pinnedConfig);
            }
            catch (exception e)
            {
                throw gcnew Exception(gcnew String(e.what()));
            }

            // Allocate a handle to the callback passed to the unmanaged code so it doesn't get 
            // garbage collected while it is being used. Don't forget to release it when we are done.
            _IntermediateCallback = gcnew IntermediateUpdaterCallbackDelegate(this, &Updater::IntermediateCallback);
            if (_IntermediateCallback)
            {
                // Convert callback delegate to unmanaged.
                _IntermediateCallbackHandle = GCHandle::Alloc(_IntermediateCallback);
                IntPtr ip = Marshal::GetFunctionPointerForDelegate(_IntermediateCallback);
                // Go ahead and register intermediate callback with unmanaged code.
                // Will check in intermediate handler if managed callback is valid or not.
                _UnmanagedUpdater->setCallback(static_cast<blfwk::Updater::progress_callback_t>(ip.ToPointer()));
            }

            // Init the managed callback to null till the user calls setCallback() with the real delegate.
            _ManagedWrapperCallback = nullptr;
        }

        ~Updater() { this->!Updater(); }
        !Updater()
        { 
            // release reference to delegate
            if (_IntermediateCallbackHandle.IsAllocated)
            {
                _IntermediateCallbackHandle.Free();
            }
            // Clean up unmanaged code.
            if ( _UnmanagedUpdater != nullptr )
                delete _UnmanagedUpdater;
        }

        //! @brief Save the managed callback delegate so we can call it from our intermediate handler.
        void setCallback(UpdaterOperationProgressDelegate^ callback)
        {
            _ManagedWrapperCallback = callback;
        }

        //! @brief The intermediate callback. Called from unmanaged code and calls managed delegate.
        void IntermediateCallback(blfwk::updater_operation_t * op)
        {
            // This is so ugly since we have access to the unmanaged progress data.
            UpdaterOperationProgressData^ opData = gcnew UpdaterOperationProgressData(op);
            if ( _ManagedWrapperCallback != nullptr )
            {
                _ManagedWrapperCallback(opData);
            }
        }

        void eraseAllUnsecure()
        {
            try
            {
                _UnmanagedUpdater->eraseAllUnsecure();
            }
            catch (const std::exception& e)
            {
                throw gcnew Exception(gcnew String(e.what()));
            }
        }

        void unlockWithKey(String ^ BackdoorKey)
        {
            try
            {
                const char* ansiStr = (const char*)(Marshal::StringToHGlobalAnsi(BackdoorKey)).ToPointer();
                _UnmanagedUpdater->unlock(ansiStr);
                Marshal::FreeHGlobal(IntPtr((void*)ansiStr));
            }
            catch (const std::exception& e)
            {
                throw gcnew Exception(gcnew String(e.what()));
            }
        }

        Boolean IsCommandSupported(String ^ Command)
        {
            try
            {
                const char* ansiStr = (const char*)(Marshal::StringToHGlobalAnsi(Command)).ToPointer();
                for each (blfwk::cmd_t var in blfwk::kCommands)
                {
                    if (((string)ansiStr).compare(var.name) == 0)
                    {
                        Marshal::FreeHGlobal(IntPtr((void*)ansiStr));
                        return _UnmanagedUpdater->isCommandSupported(var);
                    }
                }
                Marshal::FreeHGlobal(IntPtr((void*)ansiStr));
                return false;
            }
            catch (const std::exception& e)
            {
                throw gcnew Exception(gcnew String(e.what()));
            }
        }

        void flashFirmware(String^ filename, UInt32 baseAddress)
        {
            try
            {
                // Create our underlying native Worker object.
                const char* ansiStr = (const char*)(Marshal::StringToHGlobalAnsi(filename)).ToPointer();

                _UnmanagedUpdater->flashFirmware(ansiStr, baseAddress);

                Marshal::FreeHGlobal(IntPtr((void*)ansiStr));
            }
            catch (const std::exception& e)
            {
                throw gcnew Exception(gcnew String(e.what()));
            }
        }

        void reset()
        {
            try
            {
                _UnmanagedUpdater->reset();
                delete _UnmanagedUpdater;
                _UnmanagedUpdater = nullptr;
            }
            catch (const std::exception& e)
            {
                delete this;
                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Updater.reset() rethrowing: {0}"), gcnew String(e.what())));
            }
        }

        property StandardVersion SerialProtocolVersion
        {
            StandardVersion get()
            {
                blfwk::SerialPacketizer * packetizer = dynamic_cast<blfwk::SerialPacketizer *>(_UnmanagedUpdater->getPacketizer());
                if (packetizer)
                    return StandardVersion(packetizer->getVersion());

                return StandardVersion();
            }
        }

        property SecurityState TargetSecurityState
        {
            SecurityState get()
            {
                try
                {
                    return SecurityState(_UnmanagedUpdater->getSecurityState());
                }
                catch (exception e)
                {
                    throw gcnew Exception(gcnew String(e.what()));
                }
            }
        }

        property StandardVersion Version
        {
            StandardVersion get() { return StandardVersion(getVersion()); }
        }

        standard_version_t getVersion()
        {
            try
            {
                return _UnmanagedUpdater->getVersion();
            }
            catch (exception e)
            {
                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Bootloader.getVersion() rethrowing: {0}"), gcnew String(e.what())));
            }
        }

        void ping(Int32 retries, UInt32 delay, Int32 comSpeed)
        {
            try
            {
                _UnmanagedUpdater->ping(retries, delay, comSpeed);
            }
            catch (exception e)
            {
                throw gcnew Exception(String::Format(gcnew String("blfwkdll::Bootloader.ping() rethrowing: {0}"), gcnew String(e.what())));
            }
        }
    };  // class Updater

}