#include "error_handler.h"
#include "pico/stdlib.h"
#include "common/debug.h"
#include "hardware/watchdog.h"
#include "hardware/exception.h"

#define HARDFAULT_HANDLING_ASM(_x)             \
  __asm volatile(                              \
	"mov r1, lr\n"                             \
	"lsr r0, r1, #2\n"                         \
	"cmp r0, #0\n"                             \
	"bne psp_act\n"                            \
	"mrs r0, msp\n"                            \
	"b core_0_call_handler\n"                  \
	"psp_act:\n"                               \
	"mrs r0, psp\n"                            \
	"core_0_call_handler:\n"                   \
	"b core0_handler \n"                       \
	)

typedef struct __attribute__((packed)) ContextStateFrame {
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t return_address;
	uint32_t xpsr;
} sContextStateFrame;


// Default exception handler for hardfault
exception_handler_t hardfault_default_handler = NULL;


static void error_handler_set(exception_handler_t handle, enum exception_number exception);


__attribute__((optimize("O0")))
void core0_handler(sContextStateFrame *frame) {
	// If and only if a debugger is attached, execute a breakpoint
	// instruction so we can take a look at what triggered the fault
	__asm("bkpt 1");

	// Logic for dealing with the exception. Typically:
	//  - log the fault which occurred for postmortem analysis
	//  - If the fault is recoverable,
	//    - clear errors and return back to Thread Mode
	//  - else
	//    - reboot system
}

__attribute__((optimize("O0")))
void __attribute__((noreturn)) core0_hardfault_handler(void)
{
    // __asm volatile(                                
    //   "mov r1, lr\n"                             
    //   "lsr r0, r1, #2\n"                         
    //   "cmp r0, #0\n"                             
    //   "bne psp_act\n"                            
    //   "mrs r0, msp\n"                            
    //   "b core_0_call_handler\n"                  
    //   "psp_act:\n"                               
    //   "mrs r0, psp\n"                            
    //   "core_0_call_handler:\n"                   
    //   "b core0_handler \n"                       
    //                                              );
    // if (stdio_init_all())
    //     print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Hardfault at core 0, rebooting");
    watchdog_enable(1, 1);
    while (true) tight_loop_contents();
}

__attribute__((optimize("O0")))
void __attribute__((noreturn)) core1_hardfault_handler(void)
{
    // if (stdio_init_all())
    //     print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Hardfault at core 1, rebooting");
    watchdog_enable(1, 1);
    while (true) tight_loop_contents();
}

void error_handler_set_hardfault_core0(void)
{
    error_handler_set(core0_hardfault_handler, HARDFAULT_EXCEPTION);
	print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_NO_SOURCE, "Successfully assigned hardfault exception handler on core 0 to %p", exception_get_vtable_handler(HARDFAULT_EXCEPTION));
}

void error_handler_set_hardfault_core1(void)
{
	error_handler_set(core1_hardfault_handler, HARDFAULT_EXCEPTION);
	print_ser_output(SEVERITY_DEBUG, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Successfully assigned hardfault exception handler on core 1 to %p", exception_get_vtable_handler(HARDFAULT_EXCEPTION));
}

static void error_handler_set(exception_handler_t handle, enum exception_number exception)
{
	exception_handler_t current = exception_get_vtable_handler(exception); // Get current exception handler
    extern char __default_isrs_start;
  	extern char __default_isrs_end;

	if (((uintptr_t)current) >= (uintptr_t)&__default_isrs_start &&
				((uintptr_t)current) < (uintptr_t)&__default_isrs_end) // Check if currently set exception handler is default
	{
		hardfault_default_handler = exception_set_exclusive_handler(exception, handle); // Set new exception handler
	}
	else if (((uintptr_t)hardfault_default_handler) >= (uintptr_t)&__default_isrs_start && // Else check for saved default handler
			((uintptr_t)hardfault_default_handler) < (uintptr_t)&__default_isrs_end)
	{
		exception_restore_handler(exception, hardfault_default_handler); // Restore default handler
		exception_set_exclusive_handler(exception, handle); // Set new exception handler
	}
	else
	{
		print_ser_output(SEVERITY_FATAL, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to assign exception handler, resetting device..."); // Failed to assign exception handler
		watchdog_enable(1, 1);
		return;
	}
}