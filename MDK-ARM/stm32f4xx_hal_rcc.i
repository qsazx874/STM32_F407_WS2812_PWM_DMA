# 1 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"

































































 

 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


















  

 







 
# 1 "../Inc/stm32f4xx_hal_conf.h"



















  

 







 
 

 


 


   
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

 
 
 
 


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
# 88 "../Inc/stm32f4xx_hal_conf.h"

 




 












 






 







 












 





 

 


 
# 153 "../Inc/stm32f4xx_hal_conf.h"

 



 
 

 

 

 
# 172 "../Inc/stm32f4xx_hal_conf.h"

    





 

  

  

 





 



 
# 206 "../Inc/stm32f4xx_hal_conf.h"




  
 





 




 



 


 

# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

















 

 







 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"


















 

 







 
# 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"











































 



 



 
    






   


 
  


 






 
# 111 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
   


 
# 123 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 
# 135 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 



 

# 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







































 



 



 
    









 



 








 
  


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,      
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  RNG_IRQn                    = 80,      
  FPU_IRQn                    = 81       
} IRQn_Type;
 




 

# 1 "../Drivers/CMSIS/Include/core_cm4.h"
 




 
















 










# 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
# 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











# 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
# 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



# 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











# 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
# 35 "../Drivers/CMSIS/Include/core_cm4.h"

















 




 



 

# 1 "../Drivers/CMSIS/Include/cmsis_version.h"
 




 
















 










 
# 64 "../Drivers/CMSIS/Include/core_cm4.h"

 









 
# 87 "../Drivers/CMSIS/Include/core_cm4.h"

# 161 "../Drivers/CMSIS/Include/core_cm4.h"

# 1 "../Drivers/CMSIS/Include/cmsis_compiler.h"
 




 
















 




# 29 "../Drivers/CMSIS/Include/cmsis_compiler.h"



 
# 1 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 




 
















 









 













   
   


 
# 103 "../Drivers/CMSIS/Include/cmsis_armcc.h"

 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}









 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1U);
}









 
static __inline uint32_t __get_FPSCR(void)
{


  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{


  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);



}


 


 



 




 






 







 






 








 










 










 






                  





 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int16_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
# 532 "../Drivers/CMSIS/Include/cmsis_armcc.h"







 











 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 


# 780 "../Drivers/CMSIS/Include/cmsis_armcc.h"

   


 



 



# 851 "../Drivers/CMSIS/Include/cmsis_armcc.h"











 


# 35 "../Drivers/CMSIS/Include/cmsis_compiler.h"




 
# 263 "../Drivers/CMSIS/Include/cmsis_compiler.h"




# 163 "../Drivers/CMSIS/Include/core_cm4.h"

















 
# 207 "../Drivers/CMSIS/Include/core_cm4.h"

 






 
# 223 "../Drivers/CMSIS/Include/core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:1;                
    uint32_t ICI_IT_1:6;                  
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t ICI_IT_2:2;                  
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 

































 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 


















 





















 


















 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile const  uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 






 





















 






 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;



 









 









 



 









 






























 








 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
# 1562 "../Drivers/CMSIS/Include/core_cm4.h"

# 1571 "../Drivers/CMSIS/Include/core_cm4.h"









 










 


 



 





 

# 1625 "../Drivers/CMSIS/Include/core_cm4.h"

# 1635 "../Drivers/CMSIS/Include/core_cm4.h"




 
# 1646 "../Drivers/CMSIS/Include/core_cm4.h"










 
static __inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)  );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}







 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}










 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  vectors[(int32_t)IRQn + 16] = vector;
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return vectors[(int32_t)IRQn + 16];
}





 
__declspec(noreturn) static __inline void __NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 

 



# 1 "../Drivers/CMSIS/Include/mpu_armv7.h"





 
















 
 





 



# 62 "../Drivers/CMSIS/Include/mpu_armv7.h"

# 69 "../Drivers/CMSIS/Include/mpu_armv7.h"





 












   














 




  











                          









  










  












  




 




 




 




 





 
typedef struct {
  uint32_t RBAR; 
  uint32_t RASR; 
} ARM_MPU_Region_t;
    


 
static __inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

}


 
static __inline void ARM_MPU_Disable(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL  &= ~(1UL );
}



 
static __inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}




    
static __inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





    
static __inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





 
static __inline void orderedCpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i) 
  {
    dst[i] = src[i];
  }
}




 
static __inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt) 
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    orderedCpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  orderedCpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}

# 1961 "../Drivers/CMSIS/Include/core_cm4.h"




 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if      ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;            
  }
  else
  {
    return 0U;            
  }
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                               










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










# 184 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
# 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h"

































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
# 185 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
# 186 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;     
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1;
  volatile uint32_t MACDBGR;
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;





 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef;



 

typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
  uint32_t      RESERVED1;   
  uint32_t      RESERVED2;   
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED3;   
  volatile uint32_t ECCR3;       
} FSMC_Bank2_3_TypeDef;



 

typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint32_t BSRR;      
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];   
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;        
  volatile uint32_t OAR2;        
  volatile uint32_t DR;          
  volatile uint32_t SR1;         
  volatile uint32_t SR2;         
  volatile uint32_t CCR;         
  volatile uint32_t TRISE;       
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;                  
  volatile uint32_t CLKCR;                  
  volatile uint32_t ARG;                    
  volatile uint32_t CMD;                    
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;                 
  volatile uint32_t DLEN;                   
  volatile uint32_t DCTRL;                  
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;                    
  volatile uint32_t MASK;                   
  uint32_t      RESERVED0[2];           
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];          
  volatile uint32_t FIFO;                   
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  uint32_t  Reserved40[48];            
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;



 
typedef struct 
{
  volatile uint32_t DCFG;             
  volatile uint32_t DCTL;             
  volatile uint32_t DSTS;             
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;          
  volatile uint32_t DOEPMSK;          
  volatile uint32_t DAINT;            
  volatile uint32_t DAINTMSK;         
  uint32_t  Reserved20;           
  uint32_t Reserved9;             
  volatile uint32_t DVBUSDIS;         
  volatile uint32_t DVBUSPULSE;       
  volatile uint32_t DTHRCTL;          
  volatile uint32_t DIEPEMPMSK;       
  volatile uint32_t DEACHINT;         
  volatile uint32_t DEACHMSK;         
  uint32_t Reserved40;            
  volatile uint32_t DINEP1MSK;        
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;       
} USB_OTG_DeviceTypeDef;



 
typedef struct 
{
  volatile uint32_t DIEPCTL;            
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;            
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;           
  volatile uint32_t DIEPDMA;            
  volatile uint32_t DTXFSTS;            
  uint32_t Reserved18;              
} USB_OTG_INEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;            
  volatile uint32_t HCSPLT;            
  volatile uint32_t HCINT;             
  volatile uint32_t HCINTMSK;          
  volatile uint32_t HCTSIZ;            
  volatile uint32_t HCDMA;             
  uint32_t Reserved[2];            
} USB_OTG_HostChannelTypeDef;



 



 
# 938 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 





 
# 977 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 987 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 996 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1033 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 






 

 



# 1063 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"






 



   
# 1110 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 1159 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 


 


 
# 1205 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1259 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
  
 
# 1309 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1365 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1427 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 
# 1498 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1548 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1598 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1637 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 
# 1665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1721 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 1762 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1770 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
 
 
 
 
 
 
# 1812 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 1840 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1890 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 1903 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 1916 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1930 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1944 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 1989 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2000 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 2007 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 2014 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2043 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
# 2062 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2073 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2087 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2101 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2118 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2129 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2143 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2157 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2174 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

   
# 2185 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2199 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2213 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2227 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2238 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2252 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2266 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2280 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2291 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2305 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2319 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
# 2328 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2417 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2506 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2595 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2684 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
# 2783 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2881 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 2979 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3077 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3175 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3273 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3371 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3469 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3567 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3763 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3861 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 3959 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4057 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4155 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4253 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4351 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4449 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4547 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4645 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4743 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4841 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 4939 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5037 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5135 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5233 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5331 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5429 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 





 





 




 
 
 
 
 


 

 
# 5471 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 5478 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 5492 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 5508 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 5515 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 5529 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 5536 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5544 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 
# 5582 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5590 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5598 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
# 5616 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
# 5657 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5668 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5685 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 5692 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5709 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 


 
# 5728 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 






 
# 5752 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
# 5769 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5783 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5791 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5799 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 5813 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
# 5892 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 5918 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
# 5937 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
# 5999 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
# 6061 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
# 6123 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
# 6185 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 





 
 
 
 
 
 
# 6277 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6305 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6376 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6401 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6472 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6543 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6614 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6685 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
# 6703 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 6725 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6748 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6781 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6789 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 6830 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
                                             
 
# 6847 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
# 6860 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













# 6909 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6917 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













# 6966 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 6974 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













# 7023 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7031 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













# 7080 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7089 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7097 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7109 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7117 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7125 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7133 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7148 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7156 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7168 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7176 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7184 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7192 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7207 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7215 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7227 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7235 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7243 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7251 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7266 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7274 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7286 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7294 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7302 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7310 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7325 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7333 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7345 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7353 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7368 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7376 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7388 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7396 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7411 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7419 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7431 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7439 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7454 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7462 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7474 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7482 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 7499 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











# 7517 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7525 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7532 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7543 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











# 7561 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7569 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7576 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7587 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











# 7605 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7613 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7620 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7643 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7666 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7689 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7702 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7714 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7726 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7738 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7751 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7763 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7775 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7787 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7800 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7812 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7824 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7836 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7849 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7861 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7873 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7885 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7898 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7910 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7922 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7934 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7947 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7959 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7971 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 7983 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 7996 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 8008 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 8020 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 8032 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
 
 
 
 
 
# 8129 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8211 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8261 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8279 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8361 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8411 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8493 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8543 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8593 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8611 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8661 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 8678 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8776 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 8858 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 8910 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 8967 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9009 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9067 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9109 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
 
 
 
 
# 9159 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9170 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9186 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



# 9221 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
# 9233 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 9282 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9308 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9319 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 





 
 
 
 
 
 




 
# 9343 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 9356 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
 
 
 
 
 
# 9380 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9387 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9406 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
# 9432 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
 
 
 
 
 
# 9448 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9457 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9469 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9488 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 


# 9499 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9510 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9523 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 9537 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9545 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 










 










 
# 9578 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9588 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9596 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 9610 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 9626 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 










# 9644 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9651 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 9677 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9699 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 9718 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
# 9766 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9777 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 





 
# 9853 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 9888 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
# 9953 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 


 


# 9968 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 






 
# 10049 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10090 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10140 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 10159 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10170 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 10246 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10287 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10298 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 10311 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10343 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 



 
# 10360 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10374 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 10381 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
 
 
 
 
# 10395 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10412 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 


 


 
# 10465 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10509 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10579 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
# 10632 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10640 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 10653 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10723 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10793 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
# 10811 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10854 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10884 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 10912 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10960 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
# 10975 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 10987 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 



 
 
 
 
 
 






 
# 11117 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 11130 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 










# 11168 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 




 




 
# 11222 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 11230 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 11243 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 11322 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11363 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11437 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
 
 
 
 


 
# 11465 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 11472 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 11503 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11526 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11555 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 






























# 11613 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11624 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 





 



 


 
# 11656 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
# 11668 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11681 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11694 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11707 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11721 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11734 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11747 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11760 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11773 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11787 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11800 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11813 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11826 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11839 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11853 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11865 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11877 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11889 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
# 11901 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11909 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
# 11931 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

















 
# 11958 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 11965 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 11990 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 11998 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 12005 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





# 12017 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 12030 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12077 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12115 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12141 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 






# 12155 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 12162 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











# 12179 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 12186 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 







# 12206 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 12220 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 






# 12234 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 12241 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











# 12258 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 12265 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 







# 12285 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 12299 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12346 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 




 




 
# 12399 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 12424 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12434 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 12443 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 






# 12466 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
 
 
 
 
# 12504 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 12517 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12564 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12587 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











 
# 12635 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12648 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
 
 
 
 
 
# 12669 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 12677 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
# 12693 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 12701 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"






 







 





 
 
 
 
 
 
# 12733 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12747 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 12806 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 


 
# 12825 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
# 12890 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 12934 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
# 12974 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 13012 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13020 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  




 








 

  
# 13059 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13139 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13156 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13164 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
# 13194 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 13219 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 13244 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
 
 

 
# 13273 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13284 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13295 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13306 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13317 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 
 
 

 
# 13380 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 13399 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
# 13417 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 13430 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 
# 13453 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 

 
# 13520 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 
# 13554 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
   
# 13647 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13697 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13744 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13758 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 
 
 
 
 
 
# 13815 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

# 13826 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

# 13837 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 13848 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





















 
# 13879 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13899 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13913 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 13935 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 13948 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




# 13965 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 13987 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

# 14051 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14068 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


# 14084 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14110 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14126 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14138 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 14181 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 14260 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14340 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14348 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 14367 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14375 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
# 14399 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




# 14421 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14432 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14440 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14456 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14472 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 14485 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14505 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14513 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 14547 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14576 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14585 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14593 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 14634 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14642 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14656 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14691 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




# 14710 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













# 14742 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

# 14755 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14766 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 14778 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14813 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14854 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 14889 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

# 14901 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 14916 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 
# 14939 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

# 14980 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15015 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 

# 15023 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
# 15040 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
# 15053 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







# 15067 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 15075 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

# 15083 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


  



 



 

 








 


 


 


 


 
# 15133 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15144 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 


 




 


 


 


 



 





 
# 15192 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15206 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15216 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15224 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15232 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
# 15244 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15254 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15262 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15270 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15278 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15288 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15298 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 



 
# 15309 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 15372 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15384 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15392 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15406 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
# 15419 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15429 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15437 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15447 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
# 15457 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
# 15471 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
# 15478 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 



 





 
# 15495 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 




 


 





 
# 15520 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 



 



 


 


 


 







 



# 15558 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



















 
 
 
 
 
 
 
 


 




 



 



 









 
# 150 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
# 193 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 



  
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;



 




 



















 

# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


















  

 
# 297 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

 
# 251 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"









 



 
  



 
# 31 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


















 

 







 
 
 



 








 



 
# 89 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 97 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 





 



 
# 135 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 202 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 



 



 






 



 

# 238 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"













 



 
# 270 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





# 314 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






# 381 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

# 478 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

# 495 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

# 520 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 
# 539 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 


















# 589 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





# 600 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 607 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










 



 
# 631 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 640 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 647 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 759 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 819 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 
# 842 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 



 










 



 


































 



 


# 980 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 

 
# 1002 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 












 



 




























# 1058 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 















 




 
# 1099 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 









# 1129 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 



# 1167 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1177 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1196 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










# 1223 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 

























 




 








 



 




 



 
# 1303 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

# 1320 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1332 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1363 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 











 

# 1411 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

 



 



 



 
# 1439 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 





































 



 
# 1504 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
# 1519 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 

 



 







# 1546 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1557 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"
 

 



 

# 1587 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1595 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






# 1611 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 

 



 





 



 



 



 
# 1651 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 



 



 






 




 



 

 



 





 



 
# 1712 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"









 




 
# 1740 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1761 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1772 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1781 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1794 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1803 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 







 



 
# 1839 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 1854 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


# 1887 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
# 2054 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



# 2064 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 

# 2078 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 







 



 

# 2101 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

# 2129 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 










 



 














 




 




 




 







 




 
# 2207 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 
# 2251 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2265 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 







# 2538 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2552 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2769 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2783 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2790 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2811 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 2959 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 



# 2984 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3005 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3122 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3131 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3148 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3163 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






# 3192 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

















# 3218 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





# 3245 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3252 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3261 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3294 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3312 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"












# 3330 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3351 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3359 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 
# 3382 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3410 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3425 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




# 3461 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"
 




# 3491 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3498 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3510 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 

# 3524 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 
# 3545 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 











 



 












# 3618 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3627 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

# 3636 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 








# 3669 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 



 

# 3686 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




 



 
# 3720 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 
# 3747 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 



 







 

# 32 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"
# 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





# 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
# 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
# 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









# 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

# 33 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"

 



   
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

 




























 


# 103 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"







# 118 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"


 
# 140 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"




  









 


# 173 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"



  



 


# 190 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

 
 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       
 

  uint32_t PLLN;       

 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 
# 75 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
}RCC_PLLInitTypeDef;

# 176 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 202 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 293 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 378 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"





 
typedef struct
{




                                
  uint32_t PLLI2SN;    


 

  uint32_t PLLI2SR;    

 

}RCC_PLLI2SInitTypeDef;
 


 
typedef struct
{
  uint32_t PeriphClockSelection; 
 

  RCC_PLLI2SInitTypeDef PLLI2S;  
 

  uint32_t RTCClockSelection;      
 




}RCC_PeriphCLKInitTypeDef;



  

 


 



 
 
# 454 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 464 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 481 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
    
 
# 495 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 507 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 519 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 


 






 




 





 
# 548 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 



 
# 562 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 



 
# 575 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 

# 600 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
# 629 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 722 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 777 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 856 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 890 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 907 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 922 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"








 






 




# 954 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 
     
 


 
 
# 2007 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 







 
# 2099 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
# 2135 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 
# 2149 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
  






   
# 2170 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2182 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
# 2192 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
# 2203 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
  






 



                                        


# 2228 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2239 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2258 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 











# 2288 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


   
  






 
# 2307 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 




    
   






 
# 2459 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
 






  
# 2486 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2503 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
  

 
  






  
# 2556 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2563 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 
# 2580 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
  
# 2587 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
    



 
# 2603 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2612 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 








# 2635 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
   







 




  







 




 
# 2678 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2695 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 




                                          






 
                                        







 
# 2740 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2757 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 











# 2787 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
                                        







 




 
                                        







 
# 2829 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2846 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
                                        







 
# 2864 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 2871 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 

 

 
# 3256 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 3526 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 3902 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 4706 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 5735 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 5781 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




























 
# 5816 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
                             
 








 



# 5899 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"














 





# 5944 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 5967 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 6057 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
# 6076 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                 
# 6095 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6107 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 











 








 


                                 
# 6166 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6331 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
# 6380 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6614 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6669 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
# 6693 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

 

# 6720 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6733 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 

 


 



 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

# 6758 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void);







  



 
 
 
 


 




 
   
# 6792 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






 






 
# 6818 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

 





 


      



      
# 6843 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6853 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




 



 

 


 


 
# 6880 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      



























      



      


# 6934 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6942 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 6962 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 7014 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 7035 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

# 7099 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






      













 



 



  



   






 
# 34 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"



 



 

 


 



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  
 

  uint32_t LSIState;             
 

  RCC_PLLInitTypeDef PLL;         
}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 







 



 





 



 





 



 






 



 




 



 





 



 






 



 




 



 






 





 






 





 






 



 
# 236 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 







 



 
# 289 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 




 



 






 



 







 



 
# 335 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 









 
 





 


 
# 366 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 

 


 







 
# 428 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 435 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
# 452 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 459 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
# 519 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 527 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
# 545 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 553 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
# 620 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 629 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
# 648 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 657 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
# 672 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 680 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
# 696 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 705 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
# 722 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 732 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
# 750 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 757 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
# 776 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 784 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
# 804 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

# 813 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 















 









 




 



 








 




 



 





















 
# 912 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 


















 
# 955 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 



 
























 













 







 






 




 



 







 










 










 



 



 









 










 







 



 



 















 




















 




 




 











 












 













 













 




 



















 





 



 

 
 

 



 
 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);


 



 
 
void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void HAL_RCC_NMI_IRQHandler(void);

 
void HAL_RCC_CSSCallback(void);



 



 

 
 
 


 




 

 
 



 


 



 
 



 



 
 




 


 


 


 












 



 

 


 



 






















# 1413 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"































 



 



 



 







 
# 233 "../Inc/stm32f4xx_hal_conf.h"


# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"

















 

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"



 




 

 



 
typedef enum
{
  HAL_EXTI_COMMON_CB_ID          = 0x00U
} EXTI_CallbackIDTypeDef;



 
typedef struct
{
  uint32_t Line;                     
  void (* PendingCallback)(void);    
} EXTI_HandleTypeDef;



 
typedef struct
{
  uint32_t Line;      
 
  uint32_t Mode;      
 
  uint32_t Trigger;   
 
  uint32_t GPIOSel;   

 
} EXTI_ConfigTypeDef;



 

 


 



 
# 125 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"



 



 





 



 







 




 
# 183 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"



 



 

 


 



 

 


 


 








 




 




 




 








 

 


 
















# 312 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"




 

 



 




 
 
HAL_StatusTypeDef HAL_EXTI_SetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_GetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_ClearConfigLine(EXTI_HandleTypeDef *hexti);
HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti, EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void));
HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti, uint32_t ExtiLine);


 




 
 
void HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti);
uint32_t HAL_EXTI_GetPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_ClearPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_GenerateSWI(EXTI_HandleTypeDef *hexti);



 



 



 



 







 
# 237 "../Inc/stm32f4xx_hal_conf.h"


# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"



 



  

 


 



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


 

 



  



 
# 103 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"




 










  







    



 





 




 






 

 


   





 
  


 

 


 






 







 







 







 







 



 

 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 



  

 
 


 
  


 

 
# 166 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 281 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 



  








  





  






  







  






  






  





  







  






  








  





  




  






  




  


 

 
# 483 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
# 573 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 682 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
# 816 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
# 908 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
# 982 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
# 1102 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 1225 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 


  



 

 


 


 

 


 


 

 
 
 


 


 

 


 


 
# 1277 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

# 1291 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"







# 1305 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

# 1333 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 



   
 
# 1367 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 1394 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 1416 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"


 

 
# 1441 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
# 1461 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 
 




 
# 1486 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 1518 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
# 1547 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 



 

 



 



  



 

 


 



 



  



  
  






 
# 215 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  
 
 
 


 



 

 


 
# 282 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"


 

 


 



 



  



 







 
# 241 "../Inc/stm32f4xx_hal_conf.h"


# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"



 



  

 




 
   


 
typedef struct
{
  uint32_t Channel;              
 

  uint32_t Direction;            

 

  uint32_t PeriphInc;            
 

  uint32_t MemInc;               
 

  uint32_t PeriphDataAlignment;  
 

  uint32_t MemDataAlignment;     
 

  uint32_t Mode;                 


 

  uint32_t Priority;             
 

  uint32_t FIFOMode;             


 

  uint32_t FIFOThreshold;        
 

  uint32_t MemBurst;             



 

  uint32_t PeriphBurst;          



 
}DMA_InitTypeDef;




 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
  HAL_DMA_STATE_ERROR             = 0x04U,   
  HAL_DMA_STATE_ABORT             = 0x05U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER           = 0x00U,   
  HAL_DMA_HALF_TRANSFER           = 0x01U    
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID         = 0x00U,   
  HAL_DMA_XFER_HALFCPLT_CB_ID     = 0x01U,   
  HAL_DMA_XFER_M1CPLT_CB_ID       = 0x02U,   
  HAL_DMA_XFER_M1HALFCPLT_CB_ID   = 0x03U,   
  HAL_DMA_XFER_ERROR_CB_ID        = 0x04U,   
  HAL_DMA_XFER_ABORT_CB_ID        = 0x05U,   
  HAL_DMA_XFER_ALL_CB_ID          = 0x06U    
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                         

  DMA_InitTypeDef            Init;                                                               

  HAL_LockTypeDef            Lock;                                                                

  volatile HAL_DMA_StateTypeDef  State;                                                             

  void                       *Parent;                                                            

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);          

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);      

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);        
  
  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);         
  
  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);           

  volatile uint32_t              ErrorCode;                                                         
  
  uint32_t                   StreamBaseAddress;                                                 

  uint32_t                   StreamIndex;                                                       
 
}DMA_HandleTypeDef;



 

 




 




  
# 194 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 




  
# 220 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 




  





 
        



  




  




  




 




  





  




 





 




  





 




 






  




 




  




 






  




  






  




  






 




 







 




  
# 383 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 



 
 
 




 













 






 






 


 





 
# 448 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





       
# 468 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
# 488 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
# 508 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
# 528 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h"

















 

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h"



 



  

 



 
   


  
typedef enum
{
  MEMORY0      = 0x00U,     
  MEMORY1      = 0x01U      
}HAL_DMA_MemoryTypeDef;



 

 



 




 

 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 


 
         
 



 


 



 



 







 
# 641 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

 




 




 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma); 
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);


 




 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_CleanCallbacks(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



  




 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


  


  
 



 


  

 



 
# 730 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

















































  

 



 


 



  



 







 
# 245 "../Inc/stm32f4xx_hal_conf.h"

   
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"



 



  
 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
          
  uint8_t                TypeExtField;          
                  
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
# 100 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 



 





 




 







 



 




 



 




 



 




 



 




 



 




 



 





 



 
# 213 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 
   


 
# 226 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 



 
# 241 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 




 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

 
 
 
 


 



































# 347 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

# 356 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

# 385 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"






 

 



  



 
  





 

 
# 249 "../Inc/stm32f4xx_hal_conf.h"






































# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"



 



  

 


 
 


 
typedef enum 
{
  FLASH_PROC_NONE = 0U, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;



 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;    
  
  volatile uint32_t               NbSectorsToErase;    
  
  volatile uint8_t                VoltageForErase;     
  
  volatile uint32_t               Sector;              
  
  volatile uint32_t               Bank;                
  
  volatile uint32_t               Address;             
  
  HAL_LockTypeDef             Lock;                

  volatile uint32_t               ErrorCode;           

}FLASH_ProcessTypeDef;



 

 


   



  
# 97 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"


 
  


  






 




  
# 126 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"


 
  



  




   



 







  



  







  



  
  
 


 





  






  





  





  





  





  





  





  






 








 










   









   
















 















 



 

 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 

  uint32_t Banks;       
 

  uint32_t Sector;      
 

  uint32_t NbSectors;   
 

  uint32_t VoltageRange;
 

} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;   
 

  uint32_t WRPState;     
 

  uint32_t WRPSector;         
 

  uint32_t Banks;        
         

  uint32_t RDPLevel;     
 

  uint32_t BORLevel;     
 

  uint8_t  USERConfig;    

} FLASH_OBProgramInitTypeDef;



 
# 133 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 

 



 



  




 
  


  






 
  


  




 
  


  






 
  


 






  
  


  




  
  


  




  




  




     



   






 

# 249 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"



  






# 266 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



 
   
# 293 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  




     
# 311 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



  
  



 
# 327 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

# 336 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


  
    


 





# 356 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


  



 
    
# 391 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

    
# 412 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
       

  
# 430 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
# 441 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
# 451 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
# 464 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



  



 
   
# 502 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
# 524 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
     
      
  
# 543 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
# 555 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
 
 
# 566 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
# 580 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 


 
  


 
    
# 617 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
      
 
# 639 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
       

 
# 651 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
# 662 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
# 677 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



 
  


 







 



 
# 708 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



  
  
 

 


 



 
 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);

# 744 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







 



 
 
 
 


 
  




 




  





  




  




 






  






 

 


 



 



























# 849 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







# 863 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
  
# 883 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

# 898 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







# 913 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
# 928 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

# 939 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

# 949 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"













# 968 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"





  
























   


























# 1034 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



 

 


 
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);


  



  



 







 
# 298 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h"

















  

 



# 75 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h"




 
# 299 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"

 


 


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void HAL_FLASH_IRQHandler(void);
  
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
 
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);


 



 
 
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);


 



  
 
 


 



 
 


 



  



  



  



  



  




 

 


 



 






 



 

 


 



 



  



 







 
# 289 "../Inc/stm32f4xx_hal_conf.h"

 















  
























# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;



 

 


 
  


 



 



  
# 86 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


    
 


 
# 100 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


 




 




 
    


 




 



 




 



 







 



  
  
 


 





















 







 





 





 





 





 





 





 





 






 






 








 







 





 





 




 

 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 



  

  
 


 
# 66 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 
# 80 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 
# 99 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



  
  
 


 










 
# 145 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

# 193 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 

 


 
 


 
void HAL_PWREx_EnableFlashPowerDown(void);
void HAL_PWREx_DisableFlashPowerDown(void); 
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void); 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);

# 221 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

# 228 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 



 
 
 
 


 



 
 
 
 



 



 


    
 



 



 

 



   
 
 





 



 

 


 



 






# 310 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

# 321 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 



 



  



 
  







 
# 275 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"

 


 
  


 
 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);


 



 
 
 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

 
void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);

 
void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);


 



 

 
 
 


 



 



 



 
 







 



 
 
 



 



 




 



 
 
 




 



 
 


 



 
# 408 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


 



 



  



 
  







 
# 333 "../Inc/stm32f4xx_hal_conf.h"






# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"

















 

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"



 



 

 


 



 
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,   
  HAL_RTC_STATE_READY             = 0x01U,   
  HAL_RTC_STATE_BUSY              = 0x02U,   
  HAL_RTC_STATE_TIMEOUT           = 0x03U,   
  HAL_RTC_STATE_ERROR             = 0x04U    
}HAL_RTCStateTypeDef;



 
typedef struct
{
  uint32_t HourFormat;      
 

  uint32_t AsynchPrediv;    
 

  uint32_t SynchPrediv;     
 

  uint32_t OutPut;          
 

  uint32_t OutPutPolarity;  
 

  uint32_t OutPutType;      
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t Hours;            

 

  uint8_t Minutes;          
 

  uint8_t Seconds;          
 

  uint8_t TimeFormat;       
 

  uint32_t SubSeconds;     

 

  uint32_t SecondFraction;  



 

  uint32_t DayLightSaving;  
 

  uint32_t StoreOperation;  

 
}RTC_TimeTypeDef;



 
typedef struct
{
  uint8_t WeekDay;  
 

  uint8_t Month;    
 

  uint8_t Date;     
 

  uint8_t Year;     
 

}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef AlarmTime;      

  uint32_t AlarmMask;            
 

  uint32_t AlarmSubSecondMask;   
 

  uint32_t AlarmDateWeekDaySel;  
 

  uint8_t AlarmDateWeekDay;      

 

  uint32_t Alarm;                
 
}RTC_AlarmTypeDef;



 



typedef struct

{
  RTC_TypeDef                 *Instance;   

  RTC_InitTypeDef             Init;        

  HAL_LockTypeDef             Lock;        

  volatile HAL_RTCStateTypeDef    State;       

# 194 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"

}RTC_HandleTypeDef;

# 218 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"



 

 


 



 




 



 






 



 




 



 




 



 




 



 





 



 




 



 




 



 
 
# 319 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 
# 333 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 




 



 
# 355 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 




 



 
# 404 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 
# 418 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 
# 440 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 

 


 




 
# 466 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"





 










 









 






 






 






 










 










 










 












 










 











 





 





 





 





 





 





 





 





 





 







 







 





 





 



 

 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"

















  

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"



 



  

  


 



 
typedef struct 
{
  uint32_t Tamper;                      
 

  uint32_t PinSelection;                
 

  uint32_t Trigger;                     
 

  uint32_t Filter;                      
 

  uint32_t SamplingFrequency;           
 

  uint32_t PrechargeDuration;           
  

  uint32_t TamperPullUp;                
            

  uint32_t TimeStampOnTamperDetection;  
 
}RTC_TamperTypeDef;


 

 


  



 
# 105 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


  



  




 
  


 







 



 








  



  







  



  






   



  


# 177 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


 



  
# 200 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


 



  
# 215 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


 
  


  




 
  


  




 



  
# 246 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


  



  




 



  
# 268 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


  



  







 



  




  


 

  




  



  
  
 


 

 


 





 






 









 









 









 









 










 









 





 





 





 





 





 





 





 





 





 








 







 





 





 




 

 


 





 






 









 









 









 









 










 









 




 

 


 





 






 

                                                                      





 






 


                                                                      








 









 










 










 



 

 


 




 





 





 





 





 





 





 





 





 








 







 





 





 



 

 


 





 






 






 






 






 






 









 



 



 

 


 



 
 
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);

HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);
void HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);

void HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
uint32_t HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);

HAL_StatusTypeDef HAL_RTCEx_SetCoarseCalib(RTC_HandleTypeDef *hrtc, uint32_t CalibSign, uint32_t Value);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCoarseCalib(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmouthCalibMinusPulsesValue);
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);


 



 
 
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc); 
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 

 
 
 


 




 

 


 



  
# 920 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"







# 933 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"

# 970 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"













# 989 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h"


 



 



  



  
  






 
# 672 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"

 


 



 
 
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void       HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void       HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

 






 



 
 
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void                HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef   HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void         HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef* hrtc);


 



 
 
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);


 



 

 
 
 


 
 
# 761 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"






 

 


 



 
# 793 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"

# 824 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"

# 841 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h"


 



 

 


 
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t Value);
uint8_t            RTC_Bcd2ToByte(uint8_t Value);


 



 



 







 
# 341 "../Inc/stm32f4xx_hal_conf.h"


















# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

















 

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 

  uint32_t AutoReloadPreload;  
 
} TIM_Base_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCFastMode;    

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
 
} TIM_OnePulse_InitTypeDef;



 
typedef struct
{
  uint32_t  ICPolarity;  
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 
typedef struct
{
  uint32_t EncoderMode;   
 

  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 

  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



 
typedef struct
{
  uint32_t ClockSource;     
 
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;     
 
} TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;     
 
} TIM_ClearInputConfigTypeDef;



 
typedef struct
{
  uint32_t  MasterOutputTrigger;   
 
  uint32_t  MasterSlaveMode;       





 
} TIM_MasterConfigTypeDef;



 
typedef struct
{
  uint32_t  SlaveMode;         
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

} TIM_SlaveConfigTypeDef;





 
typedef struct
{
  uint32_t OffStateRunMode;      
 
  uint32_t OffStateIDLEMode;     
 
  uint32_t LockLevel;            
 
  uint32_t DeadTime;             
 
  uint32_t BreakState;           
 
  uint32_t BreakPolarity;        
 
  uint32_t BreakFilter;          
 
  uint32_t AutomaticOutput;      
 
} TIM_BreakDeadTimeConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
} HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
} HAL_TIM_ActiveChannel;



 



typedef struct

{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          

# 355 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"
} TIM_HandleTypeDef;

# 399 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"



 
 

 


 



 




 



 
# 442 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
# 457 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 





 



 




 



 






 



 







 



 





 



 




 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 




 



 







 



 






 



 




 



 





 



 
# 658 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 




 



 
# 681 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
# 700 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 







 



 
# 729 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 







 



 






 



 




 



 






 



 




 



 




 


 






 



 




 



 




 



 





 



 
# 843 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 




 



 







 



 
# 879 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
# 895 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 







 



 






 



 




 



 
# 952 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
# 966 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 






 



 
 

 


 




 
# 1016 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"





 






 






 
# 1046 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"






 
# 1063 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"






 















 















 














 














 



















 



















 
















 
















 








 







 







 






 







 










 











 
# 1275 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"








 


















 




















 

















 
















 
















 
















 




















 




















 













 












 
















 








 
 

 


 

 




 
 

 


 



# 1528 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"





































































# 1607 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"















































# 1662 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"













# 1681 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

# 1690 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"























# 1731 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

































 
 

 
# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"

















 

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"



 



 

 


 



 

typedef struct
{
  uint32_t IC1Polarity;         
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;   
 
} TIM_HallSensor_InitTypeDef;


 
 

 


 



 
# 84 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"












# 108 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"


 



 
 

 


 



 
 

 


 
# 193 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"



 
 

 


 




 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                              uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                 uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                  uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim,
                                                        TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim,
                                                TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);


 




 
 
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);


 




 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);


 
 



 



 








 
# 1769 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

 


 




 
 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

 








 




 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);







 
 



 



 







 
# 361 "../Inc/stm32f4xx_hal_conf.h"


# 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"

















 

 







 
# 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  



 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 
 

  uint32_t OverSampling;              
 
} UART_InitTypeDef;







































 
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    
 
  HAL_UART_STATE_READY             = 0x20U,    
 
  HAL_UART_STATE_BUSY              = 0x24U,    
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,    
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,    
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_UART_STATE_ERROR             = 0xE0U     
 
} HAL_UART_StateTypeDef;



 
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;         

  UART_InitTypeDef              Init;              

  uint8_t                       *pTxBuffPtr;       

  uint16_t                      TxXferSize;        

  volatile uint16_t                 TxXferCount;       

  uint8_t                       *pRxBuffPtr;       

  uint16_t                      RxXferSize;        

  volatile uint16_t                 RxXferCount;       

  DMA_HandleTypeDef             *hdmatx;           

  DMA_HandleTypeDef             *hdmarx;           

  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    gState;           

 

  volatile HAL_UART_StateTypeDef    RxState;          
 

  volatile uint32_t                 ErrorCode;         

# 188 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"

} UART_HandleTypeDef;

# 218 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"



 

 


 



 
# 240 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"


 



 




 



 




 



 





 



 






 



 





 



 




 



 




 



 




 



 




 





 
# 344 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"


 









 













 



 

 


 






 
# 400 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"





 



















 























 







 
# 465 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"






 







 







 







 

















 



















 


















 
















 



















 



















 



















 









 





 





 





 



 

 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

 







 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);



 



 
 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 
 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);


 



 
 
 
 


 


 







 

 


 
# 800 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"





 








 






 

 


 



 



 



 







 
# 365 "../Inc/stm32f4xx_hal_conf.h"

























   



























   
 
# 435 "../Inc/stm32f4xx_hal_conf.h"






 

 
# 31 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"



 



  

 
 



 



 
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


 



 
   
 


 


 
# 94 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

# 117 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


 



 





 






 





# 156 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

# 186 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


 



 





 

 



 
extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;


 

 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);


 



 
 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);







 



 
 
 


 


 
 


 


 
 
 


 



  
  






 
# 70 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"



 




 



 
 


 

 









 

 


 


 
 
 



 










































































 













 
__weak HAL_StatusTypeDef HAL_RCC_DeInit(void)
{
  return HAL_OK;
}














 
__weak HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
  uint32_t tickstart, pll_config;

   
  if(RCC_OscInitStruct == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000001U) == 0x00000001U)
  {
     
    ((void)0U);
     
    if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000004U) ||      (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000008U) && ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))))

    {
      if(((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET) && (RCC_OscInitStruct->HSEState == 0x00000000U))
      {
        return HAL_ERROR;
      }
    }
    else
    {
       
      do { if ((RCC_OscInitStruct->HSEState) == (0x1UL << (16U))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (16U)))); } else if ((RCC_OscInitStruct->HSEState) == ((uint32_t)((0x1UL << (18U)) | (0x1UL << (16U))))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (18U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (16U)))); } else { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (16U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (18U)))); } } while(0U);

       
      if((RCC_OscInitStruct->HSEState) != 0x00000000U)
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)100U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)100U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000002U) == 0x00000002U)
  {
     
    ((void)0U);
    ((void)0U);

     
    if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000000U) ||      (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000008U) && ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == 0x00000000U)))

    {
       
      if(((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET) && (RCC_OscInitStruct->HSIState != ((uint8_t)0x01)))
      {
        return HAL_ERROR;
      }
       
      else
      {
         
        ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR))) & (~((0x1FUL << (3U))))) | ((uint32_t)(RCC_OscInitStruct->HSICalibrationValue) << (3U))))));
      }
    }
    else
    {
       
      if((RCC_OscInitStruct->HSIState)!= ((uint8_t)0x00))
      {
         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x00U * 4U)) = ENABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }

         
        ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR))) & (~((0x1FUL << (3U))))) | ((uint32_t)(RCC_OscInitStruct->HSICalibrationValue) << (3U))))));
      }
      else
      {
         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x00U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000008U) == 0x00000008U)
  {
     
    ((void)0U);

     
    if((RCC_OscInitStruct->LSIState)!= ((uint8_t)0x00))
    {
       
      (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x74U) * 32U) + (0x00U * 4U)) = ENABLE);

       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x61)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x61)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x61)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x61)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
       
      (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x74U) * 32U) + (0x00U * 4U)) = DISABLE);

       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x61)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x61)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x61)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x61)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000004U) == 0x00000004U)
  {
    FlagStatus       pwrclkchanged = RESET;

     
    ((void)0U);

     
     
    if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR & ((0x1UL << (28U)))) == RESET))
    {
      do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) |= ((0x1UL << (28U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) & ((0x1UL << (28U)))); (void)tmpreg; } while(0U);
      pwrclkchanged = SET;
    }

    if((((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) & ((0x1UL << (8U)))) == 0U))
    {
       
      ((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) |= ((0x1UL << (8U))));

       
      tickstart = HAL_GetTick();

      while((((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) & ((0x1UL << (8U)))) == 0U))
      {
        if((HAL_GetTick() - tickstart) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }

     
    do { if((RCC_OscInitStruct->LSEState) == (0x1UL << (0U))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (0U)))); } else if((RCC_OscInitStruct->LSEState) == ((uint32_t)((0x1UL << (2U)) | (0x1UL << (0U))))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (2U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (0U)))); } else { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) &= ~((0x1UL << (0U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) &= ~((0x1UL << (2U)))); } } while(0U);
     
    if((RCC_OscInitStruct->LSEState) != 0x00000000U)
    {
       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
        {
          return HAL_TIMEOUT;
        }
      }
    }

     
    if(pwrclkchanged == SET)
    {
      (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR &= ~((0x1UL << (28U))));
    }
  }
   
   
  ((void)0U);
  if ((RCC_OscInitStruct->PLL.PLLState) != ((uint8_t)0x00))
  {
     
    if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) != 0x00000008U)
    {
      if((RCC_OscInitStruct->PLL.PLLState) == ((uint8_t)0x02))
      {
         
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);

         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }

         
        ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR) = ((RCC_OscInitStruct->PLL . PLLSource | RCC_OscInitStruct->PLL . PLLM | (RCC_OscInitStruct->PLL . PLLN << (6U)) | (((RCC_OscInitStruct->PLL . PLLP >> 1U) - 1U) << (16U)) | (RCC_OscInitStruct->PLL . PLLQ << (24U)))));




         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = ENABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
       
      if((RCC_OscInitStruct->PLL.PLLState) == ((uint8_t)0x01))
      {
        return HAL_ERROR;
      }
      else
      {
         
        pll_config = ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR;
        if((((pll_config) & ((0x1UL << (22U)))) != RCC_OscInitStruct->PLL.PLLSource) ||
           (((pll_config) & ((0x3FUL << (0U)))) != RCC_OscInitStruct->PLL.PLLM) ||
           (((pll_config) & ((0x1FFUL << (6U)))) != RCC_OscInitStruct->PLL.PLLN) ||
           (((pll_config) & ((0x3UL << (16U)))) != RCC_OscInitStruct->PLL.PLLP) ||
           (((pll_config) & ((0xFUL << (24U)))) != RCC_OscInitStruct->PLL.PLLQ))
        {
          return HAL_ERROR;
        }
      }
    }
  }
  return HAL_OK;
}

























 
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;

   
  if(RCC_ClkInitStruct == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);

  

 

   
  if(FLatency > ((((((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->ACR)) & ((0xFUL << (0U))))))
  {
     
    (*(volatile uint8_t *)0x40023C00U = (uint8_t)(FLatency));

    
 
    if(((((((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->ACR)) & ((0xFUL << (0U))))) != FLatency)
    {
      return HAL_ERROR;
    }
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000002U) == 0x00000002U)
  {
    
 
    if(((RCC_ClkInitStruct->ClockType) & 0x00000004U) == 0x00000004U)
    {
      (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0x7UL << (10U))))) | (0x00001C00U))));
    }

    if(((RCC_ClkInitStruct->ClockType) & 0x00000008U) == 0x00000008U)
    {
      (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0x7UL << (13U))))) | ((0x00001C00U << 3)))));
    }

    ((void)0U);
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0xFUL << (4U))))) | (RCC_ClkInitStruct->AHBCLKDivider))));
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000001U) == 0x00000001U)
  {
    ((void)0U);

     
    if(RCC_ClkInitStruct->SYSCLKSource == 0x00000001U)
    {
       
      if((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        return HAL_ERROR;
      }
    }
     
    else if((RCC_ClkInitStruct->SYSCLKSource == 0x00000002U)   ||
            (RCC_ClkInitStruct->SYSCLKSource == ((uint32_t)((0x1UL << (0U)) | (0x2UL << (0U))))))
    {
       
      if((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        return HAL_ERROR;
      }
    }
     
    else
    {
       
      if((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        return HAL_ERROR;
      }
    }

    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0x3UL << (0U))))) | ((RCC_ClkInitStruct->SYSCLKSource)))));

     
    tickstart = HAL_GetTick();

    while ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) != (RCC_ClkInitStruct->SYSCLKSource << (2U)))
    {
      if ((HAL_GetTick() - tickstart) > 5000U)
      {
        return HAL_TIMEOUT;
      }
    }
  }

   
  if(FLatency < ((((((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->ACR)) & ((0xFUL << (0U))))))
  {
      
    (*(volatile uint8_t *)0x40023C00U = (uint8_t)(FLatency));

    
 
    if(((((((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->ACR)) & ((0xFUL << (0U))))) != FLatency)
    {
      return HAL_ERROR;
    }
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000004U) == 0x00000004U)
  {
    ((void)0U);
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0x7UL << (10U))))) | (RCC_ClkInitStruct->APB1CLKDivider))));
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000008U) == 0x00000008U)
  {
    ((void)0U);
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0x7UL << (13U))))) | (((RCC_ClkInitStruct->APB2CLKDivider) << 3U)))));
  }

   
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0xFUL << (4U)))>> (4U)];

   
  HAL_InitTick (uwTickPrio);

  return HAL_OK;
}



 














 





























 
void HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
  GPIO_InitTypeDef GPIO_InitStruct;
   
  ((void)0U);
  ((void)0U);
   
  if(RCC_MCOx == 0x00000000U)
  {
    ((void)0U);

     
    do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) |= ((0x1UL << (0U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) & ((0x1UL << (0U)))); (void)tmpreg; } while(0U);

     
    GPIO_InitStruct.Pin = ((uint16_t)0x0100);
    GPIO_InitStruct.Mode = 0x00000002U;
    GPIO_InitStruct.Speed = 0x00000003U;
    GPIO_InitStruct.Pull = 0x00000000U;
    GPIO_InitStruct.Alternate = ((uint8_t)0x00);
    HAL_GPIO_Init(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), &GPIO_InitStruct);

     
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~(((0x3UL << (21U)) | (0x7UL << (24U)))))) | ((RCC_MCOSource | RCC_MCODiv)))));

    



  }

  else
  {
    ((void)0U);

     
    do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) |= ((0x1UL << (2U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) & ((0x1UL << (2U)))); (void)tmpreg; } while(0U);

     
    GPIO_InitStruct.Pin = ((uint16_t)0x0200);
    GPIO_InitStruct.Mode = 0x00000002U;
    GPIO_InitStruct.Speed = 0x00000003U;
    GPIO_InitStruct.Pull = 0x00000000U;
    GPIO_InitStruct.Alternate = ((uint8_t)0x00);
    HAL_GPIO_Init(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), &GPIO_InitStruct);

     
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~(((0x3UL << (30U)) | (0x7UL << (27U)))))) | ((RCC_MCOSource | (RCC_MCODiv << 3U))))));

    



  }

}









 
void HAL_RCC_EnableCSS(void)
{
  *(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x13U * 4U)) = (uint32_t)ENABLE;
}




 
void HAL_RCC_DisableCSS(void)
{
  *(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x13U * 4U)) = (uint32_t)DISABLE;
}






























 
__weak uint32_t HAL_RCC_GetSysClockFreq(void)
{
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
  uint32_t sysclockfreq = 0U;

   
  switch (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U)))
  {
    case 0x00000000U:   
    {
      sysclockfreq = ((uint32_t)16000000U);
       break;
    }
    case 0x00000004U:   
    {
      sysclockfreq = ((uint32_t)8000000U);
      break;
    }
    case 0x00000008U:   
    {
      
 
      pllm = ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U));
      if(((uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U)))) != 0x00000000U)
      {
         
        pllvco = (uint32_t) ((((uint64_t) ((uint32_t)8000000U) * ((uint64_t) ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1FFUL << (6U))) >> (6U))))) / (uint64_t)pllm);
      }
      else
      {
         
        pllvco = (uint32_t) ((((uint64_t) ((uint32_t)16000000U) * ((uint64_t) ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1FFUL << (6U))) >> (6U))))) / (uint64_t)pllm);
      }
      pllp = ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3UL << (16U))) >> (16U)) + 1U) *2U);

      sysclockfreq = pllvco/pllp;
      break;
    }
    default:
    {
      sysclockfreq = ((uint32_t)16000000U);
      break;
    }
  }
  return sysclockfreq;
}









 
uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}






 
uint32_t HAL_RCC_GetPCLK1Freq(void)
{
   
  return (HAL_RCC_GetHCLKFreq() >> APBPrescTable[(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x7UL << (10U)))>> (10U)]);
}






 
uint32_t HAL_RCC_GetPCLK2Freq(void)
{
   
  return (HAL_RCC_GetHCLKFreq()>> APBPrescTable[(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x7UL << (13U)))>> (13U)]);
}







 
__weak void HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
   
  RCC_OscInitStruct->OscillatorType = 0x00000001U | 0x00000002U | 0x00000004U | 0x00000008U;

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (18U))) == (0x1UL << (18U)))
  {
    RCC_OscInitStruct->HSEState = ((uint32_t)((0x1UL << (18U)) | (0x1UL << (16U))));
  }
  else if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (16U))) == (0x1UL << (16U)))
  {
    RCC_OscInitStruct->HSEState = (0x1UL << (16U));
  }
  else
  {
    RCC_OscInitStruct->HSEState = 0x00000000U;
  }

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (0U))) == (0x1UL << (0U)))
  {
    RCC_OscInitStruct->HSIState = ((uint8_t)0x01);
  }
  else
  {
    RCC_OscInitStruct->HSIState = ((uint8_t)0x00);
  }

  RCC_OscInitStruct->HSICalibrationValue = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1FUL << (3U))) >> (3U));

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR &(0x1UL << (2U))) == (0x1UL << (2U)))
  {
    RCC_OscInitStruct->LSEState = ((uint32_t)((0x1UL << (2U)) | (0x1UL << (0U))));
  }
  else if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR &(0x1UL << (0U))) == (0x1UL << (0U)))
  {
    RCC_OscInitStruct->LSEState = (0x1UL << (0U));
  }
  else
  {
    RCC_OscInitStruct->LSEState = 0x00000000U;
  }

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR &(0x1UL << (0U))) == (0x1UL << (0U)))
  {
    RCC_OscInitStruct->LSIState = ((uint8_t)0x01);
  }
  else
  {
    RCC_OscInitStruct->LSIState = ((uint8_t)0x00);
  }

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (24U))) == (0x1UL << (24U)))
  {
    RCC_OscInitStruct->PLL.PLLState = ((uint8_t)0x02);
  }
  else
  {
    RCC_OscInitStruct->PLL.PLLState = ((uint8_t)0x01);
  }
  RCC_OscInitStruct->PLL.PLLSource = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U)));
  RCC_OscInitStruct->PLL.PLLM = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U)));
  RCC_OscInitStruct->PLL.PLLN = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1FFUL << (6U))) >> (6U));
  RCC_OscInitStruct->PLL.PLLP = (uint32_t)((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3UL << (16U))) + (0x1UL << (16U))) << 1U) >> (16U));
  RCC_OscInitStruct->PLL.PLLQ = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0xFUL << (24U))) >> (24U));
}








 
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency)
{
   
  RCC_ClkInitStruct->ClockType = 0x00000001U | 0x00000002U | 0x00000004U | 0x00000008U;

   
  RCC_ClkInitStruct->SYSCLKSource = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (0U)));

   
  RCC_ClkInitStruct->AHBCLKDivider = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0xFUL << (4U)));

   
  RCC_ClkInitStruct->APB1CLKDivider = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x7UL << (10U)));

   
  RCC_ClkInitStruct->APB2CLKDivider = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x7UL << (13U))) >> 3U);

   
  *pFLatency = (uint32_t)(((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->ACR & (0xFUL << (0U)));
}





 
void HAL_RCC_NMI_IRQHandler(void)
{
   
  if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR & (((uint8_t)0x80))) == (((uint8_t)0x80))))
  {
     
    HAL_RCC_CSSCallback();

     
    (*(volatile uint8_t *) ((uint32_t)(((0x40000000UL + 0x00020000UL) + 0x3800UL) + 0x0CU + 0x02U)) = (((uint8_t)0x80)));
  }
}




 
__weak void HAL_RCC_CSSCallback(void)
{
  

 
}



 



 




 



 

 
