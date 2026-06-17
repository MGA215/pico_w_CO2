#include "error_handler.h"
#include "pico/stdlib.h"
#include "common/debug.h"
#include "hardware/watchdog.h"
#include "hardware/exception.h"
#include "pico/multicore.h"
#include <stdio.h>


typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;  // Link Register (Return address of the calling function)
    uint32_t pc;  // Program Counter (The EXACT instruction that crashed)
    uint32_t psr; // Program Status Register
} StackFrame;


// Default exception handler for hardfault
exception_handler_t hardfault_default_handler = NULL;


static void error_handler_set(exception_handler_t handle, enum exception_number exception);
void hardfault_analyzer_c(StackFrame *frame);

__attribute__((optimize("O0")))
void __attribute__((noreturn)) __time_critical_func(core0_hardfault_handler)(void)
{
    print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Hardfault at core 0");
	__asm volatile (
        "movs r0, #4          		\n" // Check bit 2 of Link Register (LR)
        "mov r1, lr           		\n"
        "tst r0, r1           		\n"
        "beq use_msp_c0       		\n"	 // If bit 2 is 0, crash happened on MSP
        "mrs r0, psp          		\n" // If bit 2 is 1, crash happened on PSP
        "b call_c_handler_c0  		\n"
        "use_msp_c0:          		\n"
        "mrs r0, msp          		\n"
        "call_c_handler_c0:   		\n"
        "ldr r1, handler_addr_c0 	\n" // Load address of our C analyzer
        "bx r1                		\n"
		".align 2             		\n"
        "handler_addr_c0: .word hardfault_analyzer_c \n"
    );
    while (true) tight_loop_contents();
}

__attribute__((optimize("O0")))
void __attribute__((noreturn)) __time_critical_func(core1_hardfault_handler)(void)
{
    print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Hardfault at core 1");
	__asm volatile (
        "movs r0, #4          		\n" // Check bit 2 of Link Register (LR)
        "mov r1, lr           		\n"
        "tst r0, r1           		\n"
        "beq use_msp_c1       		\n"	 // If bit 2 is 0, crash happened on MSP
        "mrs r0, psp          		\n" // If bit 2 is 1, crash happened on PSP
        "b call_c_handler_c1  		\n"
        "use_msp_c1:          		\n"
        "mrs r0, msp          		\n"
        "call_c_handler_c1:   		\n"
        "ldr r1, handler_addr_c1 	\n" // Load address of our C analyzer
        "bx r1                		\n"
		".align 2             		\n"
        "handler_addr_c1: .word hardfault_analyzer_c \n"
    );
    while (true) tight_loop_contents();
}

__attribute__((optimize("O0")))
void hardfault_analyzer_c(StackFrame *frame) 
{
    // Crucial debugging info:
    // uint32_t crashing_instruction_address = frame->pc;
    // uint32_t caller_function_address = frame->lr;

    printf("--- HARDFAULT DETECTED ---\n");
    printf("R0  = 0x%08lx\n", frame->r0);
    printf("R1  = 0x%08lx\n", frame->r1);
    printf("R2  = 0x%08lx\n", frame->r2);
    printf("R3  = 0x%08lx\n", frame->r3);
    printf("R12 = 0x%08lx\n", frame->r12);
    printf("LR  = 0x%08lx\n", frame->lr);
    printf("PC  = 0x%08lx\n", frame->pc);
    printf("PSR = 0x%08lx\n", frame->psr);

    // Stop execution
    while (1) {
        __breakpoint(); 
    }
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
		print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Failed to assign exception handler, resetting device..."); // Failed to assign exception handler
		watchdog_enable(1, 1);
		return;
	}
}