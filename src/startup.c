// Enable the IAR extensions for this source file.
#pragma language=extended
#pragma segment="CSTACK"
// Forward declaration of the default fault handlers.
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);
// The entry point for the application startup code.
extern void __iar_program_start(void);

extern void TIM16_IRQHandler(void);
extern void TIM17_IRQHandler(void);
extern void TIM3_IRQHandler(void);
//extern void USART1_IRQHandler(void);
extern void TIM1_IRQHandler(void);
extern void EXTI2_3_IRQHandler(void);
extern void USART3_4_IRQHandler(void);
extern void TIM7_IRQHandler(void);
extern void TIM6_IRQHandler(void);
extern void USB_IRQHandler(void);
// A union that describes the entries of the vector table.  The union is needed
// since the first entry is the stack pointer and the remainder are function
// pointers.
typedef union
{
    void (*pfnHandler)(void);
    void * ulPtr;
}
uVectorEntry;

// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
__root const uVectorEntry __vector_table[] @ ".intvec" =
{
    { .ulPtr = __sfe( "CSTACK" ) },
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                       // SWI Fault Handler
    IntDefaultHandler,                       // Bus Fault Handler
    IntDefaultHandler,                       // Usage Fault Handler
    IntDefaultHandler,                       // Reserved
    IntDefaultHandler,                       // Reserved
    IntDefaultHandler,                       // Reserved
    IntDefaultHandler,                       // Reserved
    IntDefaultHandler,                       // SVCall Handler
    IntDefaultHandler,                       // Debug Monitor Handler
    IntDefaultHandler,                       // Reserved
    IntDefaultHandler,                       // PendSV Handler
    IntDefaultHandler,                       // SysTick Handler
    //External Interrupts
    IntDefaultHandler,                      // Window WatchDog     //0                                   
    IntDefaultHandler,                      // PVD through EXTI Line detection                        
    IntDefaultHandler,                      // Tamper and TimeStamps through the EXTI line            
    IntDefaultHandler,                      // RTC Wakeup through the EXTI line                       
    IntDefaultHandler,                      // FLASH                                           
    IntDefaultHandler,                      // RCC_CRS                                            
    EXTI2_3_IRQHandler,                      // EXTI0_1                                           
    IntDefaultHandler,                      // EXTI2_3                                            
    IntDefaultHandler,                      // EXTI4_15                                            
    IntDefaultHandler,                      // DMA_CH1                                             
    IntDefaultHandler,                      // DMA_CH2_3                                             
    IntDefaultHandler,                      // DMA_CH4_5_6_7                                    
    IntDefaultHandler,                      // ADC_COMP                                   
    IntDefaultHandler,                      // TIM1_BRK_UP_TRG_COM global interrupt                                  
    TIM1_IRQHandler,                      // TIM1_CC global interrupt                                   
    IntDefaultHandler,                      // TIM2 global interrupt                                 
    TIM3_IRQHandler,                      // TIM3 global interrupt                                
  TIM6_IRQHandler,                      // TIM6_DAC global interrupt                                   
  TIM7_IRQHandler,                      // TIM7 global interrupt                          
    IntDefaultHandler,                      // TIM14 global interrupt                                             
    IntDefaultHandler,                      // TIM15 global interrupt                                             
  TIM16_IRQHandler,                       // TIM16 global interrupt                                             
  IntDefaultHandler,                      // TIM17 global interrupt                                             
    IntDefaultHandler,                      // I2C1                                    
    IntDefaultHandler,                      // I2C2              
    IntDefaultHandler,                      // SPI1                 
    IntDefaultHandler,                      // SPI2
    IntDefaultHandler,                      // USART1                                  
    IntDefaultHandler,                      // USART2                                           
  USART3_4_IRQHandler,                      // USART3_4                                           
    IntDefaultHandler,                      // TIM4                                            
    IntDefaultHandler,                      // CEC_CAN                                             
    USB_IRQHandler                       // USB                                            
};

// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
void
ResetISR(void)
{
    //
    // Call the application's entry point.
    //
    __iar_program_start();
}

// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }

}

// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}