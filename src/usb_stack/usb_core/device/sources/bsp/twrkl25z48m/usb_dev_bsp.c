/**HEADER********************************************************************
* 
* Copyright (c) 2013 Freescale Semiconductor;
* All Rights Reserved
*
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* Comments:  
*
*END************************************************************************/
#include "adapter.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
    #if (defined(CPU_MK22F51212))
    #include "MK22F51212.h"
    #include "MK22F51212_usb.h"
    #elif (defined(CPU_MK70F12))
    #include "MK70F12.h"
    #elif (defined(MCU_MKL25Z))
    #include "MKL25Z.h"
    #endif
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#include "fsl_device_registers.h"
#endif

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#define BSP_USB_INT_LEVEL                (1)

static int32_t bsp_usb_dev_io_init
(
   int32_t i
)
{
    if (i == 0)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
        //SIM_SCGC4 |= SIM_SCGC4_USBOTG_MASK;
        SIM_SCGC4_REG(SIM_BASE_PTR) |= SIM_SCGC4_USBOTG_MASK;
        //SIM_SOPT2  |= (SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK);
        SIM_SOPT2_REG(SIM_BASE_PTR) |= SIM_SOPT2_USBSRC_MASK;
        SIM_SOPT2_REG(SIM_BASE_PTR) |= SIM_SOPT2_PLLFLLSEL_MASK;
#endif
    }
    else
    {
        return -1; //unknow controller
    }

    return 0;
}

int32_t bsp_usb_dev_init(uint8_t controller_id)
{
    int32_t result = 0;

    result = bsp_usb_dev_io_init(controller_id);
    if (result != 0)
        return result;

    if (0 == controller_id)
    {
        /* Configure enable USB regulator for device */
        SIM_SOPT1CFG_REG(SIM_BASE_PTR) |= SIM_SOPT1CFG_URWE_MASK;
        SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;
        /* reset USB CTRL register */
        USB_USBCTRL_REG(USB0_BASE_PTR) = 0;
        
        /* Enable internal pull-up resistor */
        USB_CONTROL_REG(USB0_BASE_PTR) = USB_CONTROL_DPPULLUPNONOTG_MASK;
        USB_USBTRC0_REG(USB0_BASE_PTR) |= 0x40; /* Software must set this bit to 1 */

        /* setup interrupt */
        OS_intr_init(soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);
    }
    else
    {
        /* unknown controller */
        result = -1;
    }

    return result;
}
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#define BSP_USB_INT_LEVEL                (4)

static int32_t bsp_usb_dev_io_init
(
   int32_t i
)
{
    if ( i == 0)
    {
        /* Configure USB to be clocked from PLL0 */
        HW_SIM_SOPT2_CLR(SIM_SOPT2_USBFSRC_MASK);
        HW_SIM_SOPT2_SET(SIM_SOPT2_USBFSRC(1));

        /* Configure USB to be clocked from clock divider */
        HW_SIM_SOPT2_SET(SIM_SOPT2_PLLFLLSEL_MASK);
        /* Enable USB-OTG IP clocking */
        HW_SIM_SCGC4_SET(SIM_SCGC4_USBOTG_MASK);

    }
    else
    {
        return -1; //unknow controller
    }

    return 0;
}

int32_t bsp_usb_dev_init(uint8_t controller_id)
{
    int32_t result = 0;

    result = bsp_usb_dev_io_init(controller_id);
    if (result != 0)
        return result;

    /* MPU is disabled. All accesses from all bus masters are allowed */
    //  MPU_CESR=0;
    if (0 == controller_id)
    {
        /* Configure enable USB regulator for device */
        //SIM_SOPT1CFG_REG(SIM_BASE_PTR) |= SIM_SOPT1CFG_URWE_MASK;
        //SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;
        HW_SIM_SOPT1CFG_SET(SIM_SOPT1CFG_URWE_MASK);
        HW_SIM_SOPT1_SET(SIM_SOPT1_USBREGEN_MASK);

        /* reset USB CTRL register */
        //USB_USBCTRL_REG(USB0_BASE_PTR) = 0;
        USB0->USBCTRL = 0;

        /* Enable internal pull-up resistor */
        //USB_CONTROL_REG(USB0_BASE_PTR) = USB_CONTROL_DPPULLUPNONOTG_MASK;
        //USB_USBTRC0_REG(USB0_BASE_PTR) |= 0x40; /* Software must set this bit to 1 */
        HW_USB_CONTROL_WR(USB_CONTROL_DPPULLUPNONOTG_MASK);
        HW_USB_USBTRC0_SET(0x40);
        /* setup interrupt */
        OS_intr_init(USB0_IRQn, BSP_USB_INT_LEVEL, 0, TRUE);
    }
    else
    {
        /* unknown controller */
        result = -1;
    }

    return result;
}

#endif
/* EOF */
