#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>

// interrupt priority definitions
#define TIMER_INTERRUPT 0
#define IO_INTERRUPT 1
#define SYSCALL_INTERRUPT 2
#define TRAP_INTERRUPT 3

#define MAX_INTERRUPTS 4

// IVT structure
typedef struct {
    int priority;  // numeric priority of the interrupt
    void (*handler)(void);  // pointer to the interrupt handler function
} IVTEntry;

IVTEntry IVT[MAX_INTERRUPTS];

// interrupt handlers
void timerInterrupt() {
    printf("Timer Interrupt Detected!\n");
    sleep(1);
    printf("Timer Interrupt Handled.\n");
}

void ioInterrupt() {
    printf("IO Interrupt Detected!\n");
    sleep(1);
    printf("IO Interrupt Handled.\n");
}

void syscallInterrupt() {
    printf("System Call Interrupt Detected!\n");
    sleep(1);
    printf("System Call Interrupt Handled.\n");
}

void trapInterrupt() {
    printf("Trap Interrupt Detected!\n");
    sleep(1);
    printf("Trap Interrupt Handled.\n");
}

// IVT initialization
void initIVT() {
    IVT[TIMER_INTERRUPT] = (IVTEntry){1, timerInterrupt};
    IVT[IO_INTERRUPT] = (IVTEntry){2, ioInterrupt};
    IVT[SYSCALL_INTERRUPT] = (IVTEntry){3, syscallInterrupt};
    IVT[TRAP_INTERRUPT] = (IVTEntry){4, trapInterrupt};
}

// interrupt service routine (ISR)
void ISR(int interruptType) {
    if (interruptType >= 0 && interruptType < MAX_INTERRUPTS) {
        printf("Handling Interrupt Type %d with Priority %d\n", interruptType, IVT[interruptType].priority);
        IVT[interruptType].handler();
    } else {
        printf("Unknown Interrupt Type: %d\n", interruptType);
    }
}

// simulate interrupt handling
void handleInterrupts() {
    int interruptType;
    while (1) {
        printf("\nEnter interrupt type (0: Timer, 1: IO, 2: SysCall, 3: Trap, -1 to exit): ");
        scanf("%d", &interruptType);

        if (interruptType == -1) {
            printf("Exiting Interrupt Handling Loop.\n");
            break;
        }

        ISR(interruptType);
    }
}

int main() {
    printf("Initializing IVT...\n");
    initIVT();

    printf("Starting Interrupt Handling Simulation...\n");
    handleInterrupts();

    printf("Program Terminated Successfully.\n");
    return 0;
}
