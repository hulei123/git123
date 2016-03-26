/**HEADER********************************************************************
* 
* Copyright (c) 2013 - 2014 Freescale Semiconductor;
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
#include "usb_misc.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#define SIM_SOPT2_IRC48MSEL_MASK                 0x30000u
#elif ((OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)||(OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX))
#include "fsl_device_registers.h"
#endif

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#define BSP_USB_INT_LEVEL                (4)
#define USB_CLK_RECOVER_IRC_EN (*(volatile unsigned char *)0x40072144)
#define SIM_SOPT2_IRC48MSEL_MASK                 0x30000u
#define BSPCFG_USB_USE_IRC48M            (0)
#define crystal_val                       16000000
#define USBHS_USBMODE_CM_IDLE_MASK    USBHS_USBMODE_CM(0)
#define USBHS_USBMODE_CM_DEVICE_MASK  USBHS_USBMODE_CM(2)       
#define USBHS_USBMODE_CM_HOST_MASK    USBHS_USBMODE_CM(3)

static int32_t bsp_usb_dev_io_init
(
    int32_t i
)
{
    if (i == USB_CONTROLLER_KHCI_0)
    {
#if BSPCFG_USB_USE_IRC48M

        /*
        * Configure SIM_CLKDIV2: USBDIV = 0, USBFRAC = 0
        */
        SIM_CLKDIV2 = (uint32_t)0x0UL; /* Update USB clock prescalers */

        /* Configure USB to be clocked from IRC 48MHz */
        SIM_SOPT2_REG(SIM_BASE_PTR)  |= SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_IRC48MSEL_MASK;

        /* Enable USB-OTG IP clocking */
        SIM_SCGC4_REG(SIM_BASE_PTR) |= SIM_SCGC4_USBOTG_MASK;
        /* Enable IRC 48MHz for USB module */
        USB_CLK_RECOVER_IRC_EN = 0x03;
#else
        /* Configure USB to be clocked from PLL0 */
        SIM_SOPT2_REG(SIM_BASE_PTR) |= SIM_SOPT2_USBSRC_MASK;
        /* Configure USB divider to be 120MHz * 2 / 5 = 48 MHz */
        SIM_CLKDIV2_REG(SIM_BASE_PTR) = SIM_CLKDIV2_USBDIV(4) | SIM_CLKDIV2_USBFRAC_MASK;

        /* Enable USB-OTG IP clocking */
        SIM_SCGC4_REG(SIM_BASE_PTR) |= SIM_SCGC4_USBOTG_MASK;
#endif
    }
    else if (i == USB_CONTROLLER_EHCI_0)
    {   
        MCG_C1 |= MCG_C1_IRCLKEN_MASK;    //32kHz IRC enable
        OSC_CR |= OSC_CR_ERCLKEN_MASK;   //external reference clock enable

        // Configure EXT_PLL from USB HS PHY
        SIM_SOPT2 |= SIM_SOPT2_USBREGEN_MASK;// | SIM_SOPT2_PLLFLLSEL(2); //enable USB PHY PLL regulator, needs to be enabled before enable PLL
        SIM_SCGC3 |= SIM_SCGC3_USBHS_MASK | SIM_SCGC3_USBHSPHY_MASK;  //open HS USB PHY clock gate

        OSA_TimeDelay(1);
        SIM_USBPHYCTL = SIM_USBPHYCTL_USB3VOUTTRG(6) | SIM_USBPHYCTL_USBVREGSEL_MASK; //trim the USB regulator output to be 3.13V

        USBPHY_TRIM_OVERRIDE_EN = 0x001f; //override IFR value

        USBPHY_PLL_SIC |= USBPHY_PLL_SIC_PLL_POWER_MASK;  //power up PLL
        if(crystal_val == 24000000)
            USBPHY_PLL_SIC |= USBPHY_PLL_SIC_PLL_DIV_SEL(0);
        else if(crystal_val == 16000000)
            USBPHY_PLL_SIC |= USBPHY_PLL_SIC_PLL_DIV_SEL(1);
        else if(crystal_val == 12000000)
            USBPHY_PLL_SIC |= USBPHY_PLL_SIC_PLL_DIV_SEL(2);

        USBPHY_PLL_SIC &= ~USBPHY_PLL_SIC_PLL_BYPASS_MASK;   //clear bypass bit

        USBPHY_PLL_SIC |= USBPHY_PLL_SIC_PLL_EN_USB_CLKS_MASK;   //enable USB clock output from USB PHY PLL

        while (!(USBPHY_PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK))
            ;
        USBPHY_CTRL &= ~USBPHY_CTRL_SFTRST_MASK;  //release PHY from reset
        USBPHY_CTRL &= ~USBPHY_CTRL_CLKGATE_MASK; //Clear to 0 to run clocks
        USBPHY_CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK|USBPHY_CTRL_SET_ENUTMILEVEL3_MASK;

        USBPHY_PWD = 0;   //for normal operation

        USBPHY_ANACTRL |= USBPHY_ANACTRL_PFD_FRAC(24) ;//N=24

        USBPHY_ANACTRL |=USBPHY_ANACTRL_PFD_CLK_SEL(4)   //div by 4
             ;
        USBPHY_ANACTRL &= ~USBPHY_ANACTRL_DEV_PULLDOWN_MASK;
        USBPHY_ANACTRL &= ~USBPHY_ANACTRL_PFD_CLKGATE_MASK;
        while (!(USBPHY_ANACTRL & USBPHY_ANACTRL_PFD_STABLE_MASK))
            ;
        USBPHY_TX |= 1<<24; 

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
    {
        return result;
    }
    /* MPU is disabled. All accesses from all bus masters are allowed */
    MPU_CESR=0;
    if (USB_CONTROLLER_KHCI_0 == controller_id)
    {
        /* Configure enable USB regulator for device */
        SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1CFG_URWE_MASK;
        SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;

        /* reset USB CTRL register */
        USB_USBCTRL_REG(USB0_BASE_PTR) = 0;
        
        /* Enable internal pull-up resistor */
        USB_CONTROL_REG(USB0_BASE_PTR) = USB_CONTROL_DPPULLUPNONOTG_MASK;
        USB_USBTRC0_REG(USB0_BASE_PTR) |= 0x40; /* Software must set this bit to 1 */
        /* setup interrupt */
        OS_intr_init(soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);
    }
    else if (USB_CONTROLLER_EHCI_0 == controller_id)
    {
        USBHS_USBCMD |= USBHS_USBCMD_RST_MASK;
        while (USBHS_USBCMD & USBHS_USBCMD_RST_MASK)
        { /* delay while resetting USB controller */ }

        USBHS_USBMODE = USBHS_USBMODE_CM_DEVICE_MASK;
        /* Set interrupt threshold control = 0 */
        USBHS_USBCMD &= ~( USBHS_USBCMD_ITC(0xFF));
        /* Setup Lockouts Off */
        USBHS_USBMODE |= USBHS_USBMODE_SLOM_MASK;

        /* setup interrupt */
        OS_intr_init(INT_USBHS, BSP_USB_INT_LEVEL, 0, TRUE);
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
