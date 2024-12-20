#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <semaphore.h>

#define MAX_INTERRUPTS 4
#define TIMER_INT 0
#define IO_INT 1
#define SYSCALL_INT 2
#define TRAP_INT 3



struct IVTEntry {
    int priority;
    void (*handler)();
};

struct IVTEntry IVT[MAX_INTERRUPTS];



//the global variables for the CPU
int PC;
int ACC;
int IR;

//the signals used
//Mutexes and semaphores for safe access
pthread_mutex_t memory_mutex;
sem_t cache_semaphore;

//Status register flags
struct StatusRegister {
 int ZF;
 int CF;
} Status;

// Memory System
//Define cache and RAM sizes
#define L1_CACHE_SIZE 64
#define L2_CACHE_SIZE 128
#define RAM_SIZE 10000

//Define memory structures
int RAM[RAM_SIZE]; 
int L1Cache[L1_CACHE_SIZE];
int L2Cache[L2_CACHE_SIZE];
int MEMORY_TABLE_SIZE = RAM_SIZE / 100;

//to solve the issue with the update system i simply make a struct with an InRam boolean check with tags.
// this just does not update the ram value till the cache pushes the value out.
struct Tags{
    int address;
    bool inRAM;
};
struct Tags L1Tags[L1_CACHE_SIZE];
struct Tags L2Tags[L2_CACHE_SIZE];


//Structure for the Memory Table entry
struct MemoryBlock {
 int processID;
 int memoryStart;
 int memoryEnd;
 int isFree; //1 = free, 0 = allocated
};
#define MEMORY_TABLE_SIZE 10
struct MemoryBlock memoryTable[RAM_SIZE / 100];


//do - create a memo table
//Function to initialize the Memory Table
void initMemoryTable() {
 //do: Set all blocks in memoryTable as free (isFree = 1) and clear process IDs
     for (int i = 0; i < MEMORY_TABLE_SIZE; i++) {
    memoryTable[i].processID = -1; //Mark all blocks as free
    memoryTable[i].isFree = 1;
    memoryTable[i].memoryStart = i * (RAM_SIZE / MEMORY_TABLE_SIZE); // Initialize memoryStart
    memoryTable[i].memoryEnd = (i + 1) * (RAM_SIZE / MEMORY_TABLE_SIZE) - 1; // Initialize memoryEnd
    }
}
//Function for dynamic memory allocation (First-Fit or Best-Fit strategy)
int allocateMemory(int processID, int size) {
    //do: implement allocation logic to find a free block in memoryTable
    //set isFree to 0 and store processID for allocated block
     bool successful_alloc = false;
 for(int i = 0; i < RAM_SIZE / 100; i ++){
 if(memoryTable[i].isFree && (memoryTable[i].memoryEnd - memoryTable[i].memoryStart + 1) >= size){
    memoryTable[i].processID = processID;
    memoryTable[i].isFree = 0;
    successful_alloc = true;
    printf("Memory Allocation: ProcessID: %d Memory Start: %d, Memory End: %d \n", processID, memoryTable[i].memoryStart, memoryTable[i].memoryEnd);
    //sleep(1);
    break;
 }

 }
 if(!successful_alloc){
    printf("Memory Allocation Error: Not enough space for Process %d \n", processID);
    //sleep(1);
 }
    return 0;
}

//function for memory deallocation
void deallocateMemory(int processID) {
 //do: search memoryTable for blocks belonging to processID and free them (isFree = 1)
 for (int i = 0; i < RAM_SIZE / 100; i++){
    if (memoryTable[i].processID == processID){
        memoryTable[i].processID = -1;
        memoryTable[i].isFree = 1;
        printf("Memory Deallocation: ProcessID: %d Memory Start: %d Memory End: %d \n", processID, memoryTable[i].memoryStart, memoryTable[i].memoryEnd);
        break;
    }
 }
}

//int -> boolean
//purpose: to check to see if an address exist in the L1 cache
bool inCacheL1(int address){
for(int i = 0; i < L1_CACHE_SIZE; i++){
if (L1Tags[i].address == address){
    return true;
}
}
return false;
}

//int -> boolean
//purpose: to check to see if an address exist in the L2 cache
bool inCacheL2(int address){
for(int i = 0; i < L2_CACHE_SIZE; i++){
    if(L2Tags[i].address == address){
        return true;
    }
}
return false;
}

void updateCache(int address, int value, bool inRam){
    // Check if the address exists in L1 or L2 and update
    if (inCacheL1(address)) {
        for (int i = 0; i < L1_CACHE_SIZE; i++) {
            if (L1Tags[i].address == address) {
                if(!inRam){
                    L1Tags[i].inRAM = false;
                }
                L1Cache[i] = value; // Update L1 cache
                return;
            }
        }
    }

    else if (inCacheL2(address)) {
        for (int i = 0; i < L2_CACHE_SIZE; i++) {
            if (L2Tags[i].address == address) {
                if(!inRam){
                    L2Tags[i].inRAM = false;
                }
                L2Cache[i] = value; // Update L2 cache
                return;
            }
        }
    }

    else{
            // if space in cache L1
    for (int i = 0; i < L1_CACHE_SIZE; i++){
        if(L1Cache[i] == -1){
            L1Tags[i].address = address;
            L1Tags[i].inRAM = inRam;
            L1Cache[i] = value;
            return;
        }
    }

    // if space in cache L2
    for(int i = 0; i < L2_CACHE_SIZE; i++){
        if(L2Cache[i] == -1){
            L2Tags[i].address = address;
            L2Tags[i].inRAM = inRam;
            L2Cache[i] = value;
            return;
        }
    }

    // holds the values so that they do not get lost.
    int temp_value = L1Cache[0];
    struct Tags temp_address = L1Tags[0];

    //this will bump the values in l1 up 1 slot
	for(int i = 1; i < L1_CACHE_SIZE; i++){
		int bump_address = L1Cache[i];
        struct Tags bump_tag = L1Tags[i];
		L1Cache[i - 1] = bump_address;
        L1Tags[i - 1] = bump_tag;
	}

    //empty tags struct for place holding.
    struct Tags EStruct = {-1, true};
    //removes these elements from the queue
	L1Cache[L1_CACHE_SIZE - 1] = -1;
    L1Tags[L1_CACHE_SIZE - 1] = EStruct;

    //this will deal with the first element in L2.
    if(!L2Tags[0].inRAM){
        RAM[L2Tags[0].address] = L2Cache[0];
    }
    
    // does the bump for L2
    for(int i = 1; i < L2_CACHE_SIZE; i++){
        int bump_address = L2Cache[i];
        struct Tags bump_tag = L2Tags[i];
        L2Cache[i - 1] = bump_address;
        L2Tags[i - 1] = bump_tag;
    }

	// moves this element to the end of L2
    L2Cache[L2_CACHE_SIZE - 1] = temp_value;
    L2Tags[L2_CACHE_SIZE - 1] = temp_address;
	    
	//adds the new value to cache L1
	L1Cache[L1_CACHE_SIZE - 1] = value;

    struct Tags newTag = {address, inRam};
    L1Tags[L1_CACHE_SIZE - 1] = newTag;
    }
}

// Cache hits and misses count
int cacheHits;
int cacheMisses;

//handle cache lookup
int cacheLookup(int address) {
    //do: Implement L1 and L2 cache lookup logic
    //check L1 cache first; if miss, check L2 cache; if both miss, access RAM
    //return data if found or load from RAM if not found 
    //if in cache 1
    for(int i = 0; i < L1_CACHE_SIZE; i++){
        if(address == L1Tags[i].address){
            printf("L1 Cache hit value: %d \n", L1Cache[i]);
	    cacheHits += 1;
            return L1Cache[i];
        }
    }
    
    //if in cache 2
    for (int i = 0; i < L2_CACHE_SIZE; i++){
        if(address == L2Tags[i].address){
            printf("L2 Cache hit value:%d \n", L2Cache[i]);
	    cacheHits += 1;
            return L2Cache[i];
        }
    }

    // so that the cache updates cache and RAM address
    int value = RAM[address];
    cacheMisses += 1;
    updateCache(address,value,true);
    return value;
}

//function to handle cache write policies (Write-Through-0 or Write-Back-1)
void cacheWrite(int address, int data, int writePolicy) {
 //do: write data to cache and manage RAM updates based on write policy
 if(writePolicy == 0){ //write through policy
    RAM[address] = data;
    updateCache(address, data, true);
 }
 else if(writePolicy == 1){
    updateCache(address, data, false);
 }
 else{
    printf("Error: writePolicy does not exist\n");
 }
}

//function for memory access with cache (includes semaphore protection)
int accessMemory(int address, int data, bool isWrite) {
 int result;
 if (isWrite) {
	sem_wait(&cache_semaphore); //lock memory for exclusive access
	cacheWrite(address, data, 1); //use 1 for write-policy through policy in this example
	sem_post(&cache_semaphore); //unlock memory
	result = 0;
 } else {
 	sem_wait(&cache_semaphore); //lock memory for exclusive access
	result = cacheLookup(address); //fetch data from cache or RAM
	sem_post(&cache_semaphore); //unlock memory

 }
 return result;
}

//the PCB struct
struct PCB {
int pid;
int PC;
int ACC;
enum {READY, RUNNING, BLOCKED, TERMINATED} state;
int priority;
int arrivalTime;
int burstTime;
int waitingTime;
int turnTime;
int timeRemaining;
float responseRatio;
bool inQueue;
};

#define FLAGGED 1
int interruptFlag;
int carryFlag;
int zeroFlag;
int overFlowFlag;
int errorFlag;
//the amount of processes and the max processes allowed.
#define MAX_PROCESSES 6
struct PCB processTable[MAX_PROCESSES];
struct PCB* readyQueue[MAX_PROCESSES];
int readyQueueHead = 0;
int readyQueueTail = 1;
int readyQueueSize = 0;

// Vars for Feedback
struct PCB* queueHigh[MAX_PROCESSES];
struct PCB* queueMedium[MAX_PROCESSES];
struct PCB* queueLow[MAX_PROCESSES];

int highQuantum = 2;
int mediumQuantum = 4;
int lowQuantum = 6;

int highHead = 0;
int highTail = 0;
int mediumHead = 0;
int mediumTail = 0;
int lowHead = 0;
int lowTail = 0;

// Instruction Codes
#define ADD 1
#define SUB 2
#define LOAD 3
#define STORE 4
#define MUL 5
#define DIV 6
#define AND 7
#define OR 8
#define JMP 9
#define JZ 10
#define HALT 11

struct Metrics {
	clock_t exec_time;
	int context_switches;
	double avg_waiting_time;
	double avg_turn_time;
	double cpu_utilization;
	double cacheEfficiency;
};
#define MAX_ALGORITHMS 7
struct Metrics metrics_table[MAX_ALGORITHMS];

int total_switches;


// int -> int
// Purpose: gets the index value of the given pid in the process table.
int getIndex(int processID) {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		if (processTable[i].pid == processID) {
			return i;
		}
	}
}


// int, int, int, -> void()
//this will mutate the PCB with the proper values of the ACC, IR, and PC
void updatePCB(int index, int PC, int ACC){
    processTable[index].PC = PC;
    processTable[index].ACC = ACC;
}

//PCB -> void()
//does the context switch where the next processes PC, ACC, and state are changed so that the cpu runs this PCBs process.
void contextSwitch(struct PCB* nextProcess){
    PC = nextProcess->PC;
    ACC = nextProcess->ACC;
    processTable->state = RUNNING;
    printf("Context switch complete! PID %d is now running...\n", nextProcess->pid);
    return;
}

// int, int -> void
// Purpose: Save the current process state and restore the next process state for execution.
void dispatcher(struct PCB* nextProcess) {
    printf("Dispatching: Switching to process %d.\n", nextProcess->pid);
    contextSwitch(nextProcess);
}

// void -> void
// Purpose: Handle the timer interrupt, triggering a context switch for Round-Robin scheduling.
void timerInterrupt() {
    printf("Timer Interrupt: Triggering context switch...\n");
    //sleep(1);
    struct PCB* nextProcess = readyQueue[readyQueueHead];
    struct PCB* currentProcess = &processTable[readyQueueTail];
    dispatcher(nextProcess);
    readyQueueTail = (readyQueueTail + 1) % MAX_PROCESSES;
    readyQueue[readyQueueTail] = currentProcess;
    total_switches++;
}

// void -> void
// Purpose: Handle the I/O interrupt by moving a BLOCKED process to the READY state.
void ioInterrupt() {
    //sleep(1);
    printf("I/O Interrupt: Process moved from BLOCKED to READY.\n");
    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (processTable[i].state == BLOCKED) {
            processTable[i].state = READY;
            printf("Process %d is now READY.\n", processTable[i].pid);
            break;
        }
    }
}

// void -> void
// Purpose: Handle a system call interrupt to execute privileged operations like memory allocation.
void syscallInterrupt() {
    printf("System Call Interrupt: Executing privileged operation...\n");
    //sleep(1);
    int pid = rand() % MAX_PROCESSES;
    int allocSize = rand() % (RAM_SIZE / 10);
    if (allocateMemory(pid, allocSize) == 0) {
        printf("System call successful: Allocated %d units of memory to Process %d.\n", allocSize, pid);
    } else {
        printf("System call failed: Not enough memory for Process %d.\n", pid);
    }
}

// void -> void
// Purpose: Handle a trap interrupt to address errors such as division by zero and invalid memory access.
void trapInterrupt() {
    printf("Trap Interrupt: Handling error...\n");
    //sleep(1);
    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (processTable[i].state == RUNNING && errorFlag) {
            printf("Error in Process %d. Terminating...\n", processTable[i].pid);
            processTable[i].state = TERMINATED;
            deallocateMemory(processTable[i].pid);
            errorFlag = 0;
            break;
        }
    }

}

// int -> void
// Purpose: Handle interrupts by prioritizing them based on their assigned levels in the IVT.
void handleInterrupt(int interruptType) {
    static int activeInterrupt = -1;

    if (activeInterrupt == -1 || IVT[interruptType].priority < IVT[activeInterrupt].priority) {
        if (activeInterrupt != -1) {
            printf("Deferring lower-priority interrupt %d\n", activeInterrupt);
        }
        activeInterrupt = interruptType;
        IVT[interruptType].handler();
        activeInterrupt = -1;
    } else {
        printf("Interrupt %d deferred due to higher-priority active interrupt %d.\n",
               interruptType, activeInterrupt);
    }
}

// void -> void
// Purpose: Simulate and check for random interrupts, resolving them using their handlers.
void checkAndHandleInterrupts() {
    int randomInterrupt = rand() % MAX_INTERRUPTS; // Simulate random interrupt
    handleInterrupt(randomInterrupt);
}



// int -> void
// Purpose: execute an instruction in the given process
void execute(int processID){
    checkAndHandleInterrupts();
    int prev;
    int result;
    int processNum = getIndex(processID);
    //mutating the global variables so that there is a fetch decode and execute properly established.
    IR = accessMemory(PC, 1, false);

    //the decode
    int opcode = IR;
    int operand = accessMemory(PC + 1, 1, false);

    //everything else below is the execute
    switch (opcode) {
        case ADD: 
            result = ACC + operand;
            prev = ACC;
            ACC = result;
            printf("PID %d Operation Add: %d + %d = %d\n", processTable[processNum].pid, prev, operand, ACC);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
            break;
        case SUB:
            result  = ACC - operand;
            prev = ACC;
            ACC = result;
            printf("PID %d Operation Subtracting: %d - %d = %d\n", processTable[processNum].pid, prev, operand, ACC);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
            break;
        case MUL:
            prev = ACC;
            result = ACC * operand;
            ACC = result;
            printf("PID %d Operation Multiplying: %d * %d = %d\n", processTable[processNum].pid, prev, operand, ACC);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
            break;
        case DIV:
            if(operand != 0){
                prev = ACC;
                result = ACC / operand;
                ACC = result;
                printf("PID %d Operation Dividing: %d / %d = %d\n", processTable[processNum].pid, prev, operand, ACC);
                PC += 2; //Move to the next instruction
                updatePCB(processNum, PC, ACC);
                break;
            }
            else{
                printf("PID %d Operation Division Error: Division by zero\n", processTable[processNum].pid);
                errorFlag = FLAGGED;
                break;
            }
        case LOAD:
            prev = ACC;
            ACC = operand;
			printf("PID %d Operation Loading data in ACC: %d -> %d\n",processTable[processNum].pid , prev, ACC);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
			break;
        case STORE:
            //(int address, int data, bool isWrite)
            accessMemory(operand, ACC, true);
            printf("PID %d Operation Storing data into memory from ACC to Adress: %d -> %d\n", processTable[processNum].pid, ACC, operand);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
			break;
        case AND:
            prev = ACC;
            result = ACC & operand;
            ACC = result;
			printf("PID %d Operation AND operation: %d & %d = %d \n", processTable[processNum].pid,  prev, operand, ACC);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
            break;
        case OR:
            prev = ACC;
            result = ACC | operand;
            ACC = result;
            printf("PID %d Operation OR operation: %d | %d = %d \n", processTable[processNum].pid , prev, operand, ACC);
            PC += 2; //Move to the next instruction
            updatePCB(processNum, PC, ACC);
            break;
        case JMP:
            if (operand >= 0  && operand < RAM_SIZE){
                prev = PC;
                PC = operand - 2;
                printf("PID %d Operation Jump: PC has been changed from %d to %d\n",processTable[processNum].pid ,  prev, PC + 2);
                PC += 2; //Move to the next instruction
                updatePCB(processNum, PC, ACC);
                break;
            }
            else{
                printf("PID %d Operation Error: Invalid Jump attempt. Out of Bounds Error\n", processTable[processNum].pid);
                errorFlag = FLAGGED;
                break;
            }
        case JZ:
            if(zeroFlag == 1){
                if(operand >= 0  && operand < RAM_SIZE){
                    prev = PC;
                    PC = operand - 2;
                    printf("PID %d Operation Jump Zero: PC has been changed from %d to %d\n", processTable[processNum].pid, prev, PC);
                    PC += 2; //Move to the next instruction
                    updatePCB(processNum, PC, ACC);
                    break;
                    }
                    else{
                        printf("PID %d Operation Error: Invalid Jump attempt. Out of Bounds Error\n",processTable[processNum].pid);
                        errorFlag = FLAGGED;
                        break;
                    }
                
            }
            else{
                printf("PID %d Operation Jump Zero: Unsuccessful jump, no zero flag present.\n", processTable[processNum].pid);
                PC += 2;
                updatePCB(processNum, PC, ACC);
                break;
            }
		default:
			// Handle undefined/invalid opcodes
			printf("PID: %d Operation: Invalid opcode given. Please try again.\n", processTable[processNum].pid);
            errorFlag = 1;
			break;
        }
        //sleep(1);

    }



#define BUFFER_SIZE 5 //buffer size for producer-consumer
#define NUM_THREADS 4 //no of threads for each main task
sem_t buffer_full, buffer_empty;

//shared buffer for producer-consumer example
int buffer[BUFFER_SIZE];
int buffer_index = 0;

//Producer-Consumer Task: Producer function
void* producer(void* arg) {
    while (1) {
        int data = rand() % 100; // Generate some random data

        sem_wait(&buffer_empty); // Wait if buffer is full

        // Add data to the buffer
        buffer[buffer_index] = data;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE; // Circular buffer logic
        printf("Produced: %d\n", data);

        sem_post(&buffer_full); // Signal buffer has new data

        sleep(1); // Simulate production delay
    }
    return NULL;
}

//Producer-Consumer Task: Consumer function
void* consumer(void* arg) {
  while (1) {
        sem_wait(&buffer_full); // Wait if buffer is empty

        // Remove data from the buffer
        buffer_index = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE; // Circular buffer logic
        int data = buffer[buffer_index];
        printf("Consumed: %d\n", data);

        sem_post(&buffer_empty); // Signal buffer has space

        sleep(1); // Simulate consumption delay
    }
    return NULL;
}

// -> void
// Purpose: load instructions and values into RAM to initialize the program.
void loadProgram() {
    // Complete: Load sample instructions into RAM
    RAM[0] = LOAD;   RAM[1] = 10;    
    RAM[2] = ADD;    RAM[3] = 5;     
    RAM[4] = STORE;  RAM[5] = 3;     
    RAM[6] = LOAD;   RAM[7] = 12;    
    RAM[8] = SUB;    RAM[9] = 3;     
    RAM[10] = JZ;    RAM[11] = 16;   
    RAM[12] = JMP;   RAM[13] = 16;   
    RAM[14] = MUL;   RAM[15] = 2;    
    RAM[16] = LOAD;  RAM[17] = 2;    
    RAM[18] = DIV;   RAM[19] = 2;    
    RAM[20] = LOAD;  RAM[21] = 3;     
    RAM[22] = ADD;   RAM[23] = 10;    
    RAM[24] = AND;   RAM[25] = 4;     
    RAM[26] = OR;    RAM[27] = 8;    
    RAM[28] = STORE; RAM[29] = 25;    
    RAM[30] = LOAD;  RAM[31] = 7;     
    RAM[32] = JMP;   RAM[33] = 36;    
    RAM[34] = DIV;   RAM[35] = 2;     
    RAM[36] = SUB;   RAM[37] = 1;     
    RAM[38] = LOAD;  RAM[39] = 15;
    RAM[40] = ADD;   RAM[41] = 25;
    RAM[42] = MUL;   RAM[43] = 35;
    RAM[44] = STORE; RAM[45] = 45;
    RAM[46] = DIV;   RAM[47] = 55;
    RAM[48] = JMP;   RAM[49] = 52;
    RAM[50] = JZ;    RAM[51] = 40;
    RAM[52] = LOAD;  RAM[53] = 65;
    RAM[54] = SUB;   RAM[55] = 10;
    RAM[56] = OR;    RAM[57] = 85;
    RAM[58] = LOAD;  RAM[59] = 10;
    RAM[60] = ADD;   RAM[61] = 40;
    RAM[62] = SUB;   RAM[63] = 30;
    RAM[64] = MUL;   RAM[65] = 40;
    RAM[66] = DIV;   RAM[67] = 50;
    RAM[68] = STORE; RAM[69] = 61;
    RAM[70] = JMP;   RAM[71] = 72;
    RAM[72] = JZ;    RAM[73] = 80;
    RAM[74] = AND;   RAM[75] = 90;
    RAM[76] = OR;    RAM[77] = 100;
    RAM[78] = LOAD;  RAM[79] = 10;    
    RAM[80] = ADD;   RAM[81] = 5;     
    RAM[82] = STORE; RAM[83] = 3;     
    RAM[84] = LOAD;  RAM[85] = 12;    
    RAM[86] = SUB;   RAM[87] = 3;     
    RAM[88] = JZ;    RAM[89] = 16;   
    RAM[90] = JMP;   RAM[91] = 100;   
    RAM[92] = MUL;   RAM[93] = 2;    
    RAM[94] = LOAD;  RAM[95] = 2;    
    RAM[96] = DIV;   RAM[97] = 2;    
    RAM[98] = LOAD;  RAM[99] = 3;     
    RAM[100] = ADD;  RAM[101] = 10;    
    RAM[102] = AND;  RAM[103] = 4;     
    RAM[104] = OR;   RAM[105] = 8;    
    RAM[106] = STORE;RAM[107] = 25;    
    RAM[108] = LOAD; RAM[109] = 7;     
    RAM[110] = JMP;  RAM[111] = 120;    
    RAM[112] = DIV;  RAM[113] = 2;     
    RAM[114] = SUB;  RAM[115] = 1;     
    RAM[116] = LOAD; RAM[117] = 15; 
    RAM[118] = ADD;  RAM[119] = 25;
    RAM[120] = MUL;  RAM[121] = 35;
    RAM[122] = STORE;RAM[123] = 45;
    RAM[124] = DIV;  RAM[125] = 55;
    RAM[126] = JMP;  RAM[127] = 142;
    RAM[128] = JZ;   RAM[129] = 40;
    RAM[130] = LOAD; RAM[131] = 65;
    RAM[132] = SUB;  RAM[133] = 75;
    RAM[134] = OR;   RAM[135] = 85;
    RAM[136] = LOAD; RAM[137] = 10;
    RAM[138] = ADD;  RAM[139] = 40;
    RAM[140] = SUB;  RAM[141] = 30;
    RAM[142] = MUL;  RAM[143] = 40;
    RAM[144] = DIV;  RAM[145] = 50;
    RAM[146] = STORE;RAM[147] = 61;
    RAM[148] = JMP;  RAM[149] = 154;
    RAM[150] = JZ;   RAM[151] = 80;
    RAM[152] = AND;  RAM[153] = 90;
    RAM[154] = JMP;   RAM[155] = 0;
}


// -> void
// Purpose: initialize the cache
void initCache(){
   struct Tags initTag = {-1, true};
    //initialize an empty cache l1
    for(int i = 0; i < L1_CACHE_SIZE; i++){
        L1Cache[i] = -1;
        L1Tags[i] = initTag; 
    }
    //empty cache l2
    for(int i = 0; i < L2_CACHE_SIZE; i++){
        L1Cache[i] = -1;
        L2Tags[i] = initTag;
    }
}

// -> void
// Purpose: initialize the process table
void initProcesses(){
	int burstTimes[] = {200, 100, 30, 125, 70};
    int arrivalTimes[] = {26, 50, 0, 19, 60, 4};
    int priorities[] = {6, 3, 5, 4, 2, 1};
    int PCs[] = {0, 10, 0, 112, 58, 116};
	for(int i = 0; i < MAX_PROCESSES; i++){
		processTable[i].pid = i + 100;
		processTable[i].PC = PCs[i];
		processTable[i].ACC = 0;
		processTable[i].state = READY;
		processTable[i].priority = priorities[i];
		processTable[i].arrivalTime = arrivalTimes[i];
		processTable[i].burstTime = burstTimes[i];
		processTable[i].waitingTime = 0;
		processTable[i].turnTime = 0;
		processTable[i].timeRemaining = processTable[i].burstTime;
		processTable[i].responseRatio = 0;
		processTable[i].inQueue = false;

        allocateMemory(processTable[i].pid, processTable[i].burstTime);
	}
}



// -> void
// Purpose: sort the processes in the ready queue by their arrival times
void sortProcessesByArrival() {
    for (int i = 0; i < MAX_PROCESSES; i++) {
        readyQueue[i] = &processTable[i];
	processTable[i].inQueue = true;
    }
    for (int i = 0; i < MAX_PROCESSES - 1; i++) {
        for (int j = 0; j < MAX_PROCESSES - i - 1; j++) {
            if (readyQueue[j]->arrivalTime > readyQueue[j + 1]->arrivalTime) {
                struct PCB* temp = readyQueue[j];
                readyQueue[j] = readyQueue[j + 1];
                readyQueue[j + 1] = temp;
            }
        }
    }
}

// -> void
// Purpose: get the first arrived process and add it to the queue
void firstProcessInQueue() {
	struct PCB* first = &processTable[0];
	for (int i = 1; i < MAX_PROCESSES; i++) {
		if (processTable[i].arrivalTime < first->arrivalTime) {
			first = &processTable[i];
		}
	}
	readyQueueSize = 1;
	first->inQueue = true;
	readyQueue[0] = first;
}

// int, bool -> void
// Purpose: check for any arrived processes and add it to the queue
void checkForNewProcesses(int time, bool feedback) {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		struct PCB* process = &processTable[i];
		if (process->inQueue) {
			continue;
		}
		if (process->arrivalTime <= time) {
			if (feedback) {
				queueHigh[highTail] = process;
				highTail = (highTail + 1) % MAX_PROCESSES;
				process->inQueue = true;
			} else {
				readyQueue[readyQueueTail] = process;
				readyQueueSize += 1;
				process->inQueue = true;
				readyQueueTail = (readyQueueTail + 1) % MAX_PROCESSES;
			}
		}
	}
}

// -> void
// Purpose: sort the ready queue by the priority values and then by arrival times
void sortByPriority() {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		readyQueue[i] = &processTable[i];
	}
    for (int i = 0; i < MAX_PROCESSES - 1; i++) {
        for (int j = i + 1; j < MAX_PROCESSES; j++) {
            if (readyQueue[i]->priority < readyQueue[j]->priority) {
                struct PCB* temp = readyQueue[i];
                readyQueue[i] = readyQueue[j];
                readyQueue[j] = temp;
            }
            else if (readyQueue[i]->priority == readyQueue[j]->priority &&
                     readyQueue[i]->arrivalTime > readyQueue[j]->arrivalTime) {
                struct PCB* temp = readyQueue[i];
                readyQueue[i] = readyQueue[j];
                readyQueue[j] = temp;
            }
        }
    }
}

// int -> void
// Purpose: Find the first process in queue and assign the head to that index. Helper for priority scheduler.
void pickPriorityProcess(int currentTime) {
	if (currentTime == 0) {
		return;
	}
	for (int i = 0; i < MAX_PROCESSES; i++) {
		if (readyQueue[i]->arrivalTime <= currentTime && (readyQueue[i]->state != TERMINATED && readyQueue[i]->state != BLOCKED)) {
				readyQueueHead = i;
				readyQueue[i]->inQueue = true;
				return;
		}
	}
}

// int -> void
// Purpose:Sort the ready queue by their burst time and then their arrival time.
void sortByShortestBurstTime(int currentTime) {
    int readyIndex = 0;

    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (processTable[i].arrivalTime <= currentTime && processTable[i].state != TERMINATED) {
            readyQueue[readyIndex++] = &processTable[i];
        }
    }

    for (int i = 0; i < readyIndex - 1; i++) {
        for (int j = i + 1; j < readyIndex; j++) {
            if (readyQueue[i]->burstTime > readyQueue[j]->burstTime) {
                struct PCB* temp = readyQueue[i];
                readyQueue[i] = readyQueue[j];
                readyQueue[j] = temp;
            }
            else if (readyQueue[i]->burstTime == readyQueue[j]->burstTime &&
                     readyQueue[i]->arrivalTime > readyQueue[j]->arrivalTime) {
                struct PCB* temp = readyQueue[i];
                readyQueue[i] = readyQueue[j];
                readyQueue[j] = temp;
            }
        }
    }
    readyQueueSize = readyIndex;
}

// -> bool
// Purpose: check if the process table is complete.
bool complete_processes() {
        for (int i = 0; i < MAX_PROCESSES; i++) {
                if (processTable[i].state != TERMINATED) {
                        return false;
                }
        }
        return true;
}
//int time, int PID -> void()
//to execute a process n amount of times
void executeProcess(int time, int processPID){
    for(int i = 0; i < time; i++){
        execute(processPID);
    }
}

// int* -> void
// Purpose: Process the high priority queue in feedback
int processHighQueue(int *currentTime) {
    int processCompleted = 0;
	while (queueHigh[highHead] != NULL) {
		struct PCB* currentProcess = queueHigh[highHead];
		int executeTime = (currentProcess->timeRemaining > highQuantum) ? highQuantum : currentProcess->timeRemaining;
        contextSwitch(currentProcess);
        executeProcess(executeTime, currentProcess->pid);
		printf("High: P%d [%d - %d]\n", currentProcess->pid, *currentTime, *currentTime + executeTime);
		*currentTime += executeTime;
		currentProcess->timeRemaining -= executeTime;
		if (currentProcess->timeRemaining <= 0) {
			currentProcess->state = TERMINATED;
            processCompleted++;
                deallocateMemory(currentProcess->pid);
	    		currentProcess->turnTime = *currentTime - currentProcess->arrivalTime;
	    		currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
	    		printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n", currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
			total_switches += 1;
		} else {
	    		currentProcess->state = READY;
	     		queueMedium[mediumTail] = currentProcess;
			mediumTail = (mediumTail + 1) % MAX_PROCESSES;
	    		printf("Process %d moved to Medium Queue\n", currentProcess->pid);
			total_switches += 1;
	 	}
		queueHigh[highHead] = NULL;
		highHead = (highHead + 1) % MAX_PROCESSES;
    	}
        return processCompleted;
}

// int* -> void
// Purpose: process the medium priority queue in feedback
int processMediumQueue(int *currentTime) {
    int processCompleted = 0;
	while (queueMedium[mediumHead] != NULL) {
		struct PCB *currentProcess = queueMedium[mediumHead];
		int executeTime = (currentProcess->timeRemaining > mediumQuantum) ? mediumQuantum : currentProcess->timeRemaining;
        contextSwitch(currentProcess);
		executeProcess(executeTime, currentProcess->pid);
		printf("Medium: P%d [%d - %d]\n", currentProcess->pid, *currentTime, *currentTime + executeTime);
		*currentTime += executeTime;
	       	currentProcess->timeRemaining -= executeTime;
		if (currentProcess->timeRemaining <= 0) {
	    		currentProcess->state = TERMINATED;
                deallocateMemory(currentProcess->pid);
                processCompleted++;
	    		currentProcess->turnTime = *currentTime - currentProcess->arrivalTime;
	    		currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
			total_switches += 1;
		} else if (currentProcess->timeRemaining <= highQuantum) {
			currentProcess->state = READY;
			queueHigh[highTail] = currentProcess;
			highTail = (highTail + 1) % MAX_PROCESSES;
			queueMedium[mediumHead] = NULL;
			mediumHead = (mediumHead + 1) % MAX_PROCESSES;
			total_switches += 1;
			return processCompleted;
		} else if (currentProcess->timeRemaining <= mediumQuantum) {
			currentProcess->state = READY;
			queueMedium[mediumTail] = currentProcess;
			mediumTail = (mediumTail + 1) % MAX_PROCESSES;
			total_switches += 1;
		} else {
			currentProcess->state = READY;
			queueLow[lowTail] = currentProcess;
			lowTail = (lowTail + 1) % MAX_PROCESSES;
			total_switches += 1;
		}
		queueMedium[mediumHead] = NULL;
		mediumHead = (mediumHead + 1) % MAX_PROCESSES;
	}
    return processCompleted;
}

// int* -> void
// Purpose: process the low priority queue in feedback
int processLowQueue(int *currentTime) {     
    int processCompleted = 0;                     
   	while (queueLow[lowHead] != NULL) {                                       
      		struct PCB *currentProcess = queueLow[lowHead];     
		int executeTime = (currentProcess->timeRemaining > lowQuantum) ? lowQuantum : currentProcess->timeRemaining;
                contextSwitch(currentProcess);
                executeProcess(executeTime, currentProcess->pid);
		printf("Low: P%d [%d - %d]\n", currentProcess->pid, *currentTime, *currentTime + executeTime);
                *currentTime += executeTime;
                currentProcess->timeRemaining -= executeTime;
                if (currentProcess->timeRemaining <= 0) {
                        currentProcess->state = TERMINATED;
                        deallocateMemory(currentProcess->pid);
                        processCompleted++;
                        currentProcess->turnTime = *currentTime - currentProcess->arrivalTime;
			currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
			total_switches += 1;
		} else if (currentProcess->timeRemaining <= highQuantum) {
                        currentProcess->state = READY;                                                      
                        queueHigh[highTail] = currentProcess;
                        highTail = (highTail + 1) % MAX_PROCESSES;
		        queueLow[lowHead] = NULL;	
			lowHead = (lowHead + 1) % MAX_PROCESSES;
			total_switches += 1;
                        return processCompleted;
                } else if (currentProcess->timeRemaining <= mediumQuantum) {
                        currentProcess->state = READY;
                        queueMedium[mediumTail] = currentProcess;
                        mediumTail = (mediumTail + 1) % MAX_PROCESSES;
			queueLow[lowHead] = NULL;
			lowHead = (lowHead + 1) % MAX_PROCESSES;
			total_switches += 1;
			return processCompleted;
                } else {
                        currentProcess->state = READY;
                        queueLow[lowTail] = currentProcess;
                        lowTail = (lowTail + 1) % MAX_PROCESSES;
			total_switches += 1;
		}
		queueLow[lowHead] = NULL;
		lowHead = (lowHead + 1) % MAX_PROCESSES;
	}
    return processCompleted;
}

// -> void
// Purpose: scheduler for feedback algorithm
void Feedback_Scheduler() {
	int currentTime = 0;
    int completed = 0;
	while (completed < MAX_PROCESSES) {
		checkForNewProcesses(currentTime, true);

		if (queueHigh[highHead] == NULL && queueMedium[mediumHead] == NULL && queueLow[lowHead] == NULL) {
			currentTime++;
			continue;
		}
		completed+= processHighQueue(&currentTime);
		checkForNewProcesses(currentTime, true);
		if (queueHigh[highHead] != NULL) {
			continue;
		}
		completed+=processMediumQueue(&currentTime);
		checkForNewProcesses(currentTime, true);
		if (queueHigh[highHead] != NULL) {
			continue;
		}
		completed+=processLowQueue(&currentTime);
		checkForNewProcesses(currentTime, true);
		if (queueHigh[highHead] != NULL || queueMedium[mediumHead] != NULL) {
			continue;
		}
	}
}

// -> void
// Purpose: scheduler for HRRN
void HRRN_Scheduler() {
    int currentTime = 0;
    int completed = 0;

    while (completed < MAX_PROCESSES) {
        struct PCB* selectedProcess = NULL;
        float highestResponseRatio = 0.0;
	checkForNewProcesses(currentTime, false);

        for (int i = 0; i < MAX_PROCESSES; i++) {
            struct PCB* process = readyQueue[i];
	    if (!process) {
		    continue;
	    }

            if (process->state != TERMINATED && process->state != BLOCKED && process->arrivalTime <= currentTime) {
                int waitingTime = currentTime - process->arrivalTime;
                float responseRatio = (float)(waitingTime + process->burstTime) / process->burstTime;
		process->responseRatio = responseRatio;
                if (responseRatio > highestResponseRatio) {
                    highestResponseRatio = responseRatio;
                    selectedProcess = process;
                }
            }
        }

        if (!selectedProcess) {
            currentTime++;
            continue;
        }

        printf("Process %d selected with response ratio %.2f at time %d\n", selectedProcess->pid, highestResponseRatio, currentTime);
        contextSwitch(selectedProcess);
        executeProcess(selectedProcess->burstTime, selectedProcess->pid);
        currentTime += selectedProcess->burstTime;

        selectedProcess->state = TERMINATED;
        deallocateMemory(selectedProcess->pid);
        selectedProcess->turnTime = currentTime - selectedProcess->arrivalTime;
        selectedProcess->waitingTime = selectedProcess->turnTime - selectedProcess->burstTime;
        completed++;
	total_switches += 1;

        printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n",
               selectedProcess->pid, selectedProcess->waitingTime, selectedProcess->turnTime);
    }
}

// -> void
// Purpose: scheduler for SRT
void SRT_Scheduler() {
    int currentTime = 0;
    int completed = 0;
    struct PCB* currentProcess = NULL;

    while (completed < MAX_PROCESSES) {
        struct PCB* shortestProcess = NULL;

        for (int i = 0; i < MAX_PROCESSES; i++) {
            if (processTable[i].state != TERMINATED && processTable[i].state != BLOCKED && processTable[i].arrivalTime <= currentTime) {
                if (!shortestProcess || processTable[i].timeRemaining < shortestProcess->timeRemaining) {
                    shortestProcess = &processTable[i];
                }
            }
        }

        if (!shortestProcess) {
            currentTime++;
            continue;
        }

        if (currentProcess != shortestProcess) {
            currentProcess = shortestProcess;
            currentProcess->state = READY;
            contextSwitch(currentProcess);
            printf("New shortest process %d changed at time %d\n", currentProcess->pid, currentTime);
        }

        execute(currentProcess->pid);
	printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + 1);
        currentProcess->timeRemaining--;
        currentTime++;
	total_switches += 1;

        if (currentProcess->timeRemaining <= 0) {
            currentProcess->state = TERMINATED;
            deallocateMemory(currentProcess->pid);
            currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
            currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
            completed++;
            printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n",
                   currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
            currentProcess = NULL;
        }
    }
}

// -> void
// Purpose: scheduler for SPN
void SPN_Scheduler() {
    int currentTime = 0;
    int completed = 0;

    while (completed < MAX_PROCESSES) {
        sortByShortestBurstTime(currentTime);

        if (readyQueueSize == 0) {
            currentTime++;
            continue;
        }

        struct PCB* currentProcess = readyQueue[0];
        contextSwitch(currentProcess);
        executeProcess(currentProcess->burstTime, currentProcess->pid);
	printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + currentProcess->burstTime);
        currentTime += currentProcess->burstTime;

        currentProcess->timeRemaining = 0;
        currentProcess->state = TERMINATED;
        deallocateMemory(currentProcess->pid);
        currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
        currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
        completed++;
	total_switches += 1;
        printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n",
               currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
    }
}	

// -> void
// Purpose: scheduler for priority
void Priority_Scheduler() {
    int currentTime = 0;
    int completed = 0;
    int prevProcessPID = -1;
    firstProcessInQueue();
    while (completed < MAX_PROCESSES) {
	    sortByPriority();
	    pickPriorityProcess(currentTime);
        struct PCB* currentProcess = readyQueue[readyQueueHead];
        int currentProcessPID = currentProcess->pid; 

        if (currentTime < currentProcess->arrivalTime) {
		currentTime++;
		continue;
	}
    if(prevProcessPID != currentProcessPID){
        contextSwitch(currentProcess);
        prevProcessPID = currentProcessPID;
    }

	execute(currentProcess->pid);

	printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + 1);
        currentProcess->timeRemaining -= 1;
        currentTime += 1;
        if (currentProcess->timeRemaining <= 0) {
		currentProcess->state = TERMINATED;
        deallocateMemory(currentProcess->pid);
		currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
		currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
            completed++;
            printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n",
                currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
            readyQueueHead = (readyQueueHead + 1) % MAX_PROCESSES;
        }
	checkForNewProcesses(currentTime, false);
	total_switches += 1;
    }
}

// int -> void
// Purpose: scheduler for round robin algorithm
void RoundRobin_Scheduler(int timeQuantum) {
	int currentTime = 0;
	int completed = 0;
    int prevProcessPID = -1;
	firstProcessInQueue();
	while (completed < MAX_PROCESSES) {
		int head = readyQueueHead;
		int tail = readyQueueTail;
		struct PCB* currentProcess = readyQueue[head];
        int currentProcessPID = currentProcess->pid;
		if (currentTime < currentProcess->arrivalTime) {
			currentTime++;
			continue;
		}
        
		int timeSlice = (currentProcess->timeRemaining < timeQuantum) ? currentProcess->timeRemaining : timeQuantum;
        if(prevProcessPID != currentProcessPID){
        contextSwitch(currentProcess);
        prevProcessPID = currentProcessPID;
        }
        executeProcess(timeSlice, currentProcess->pid);
		printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + timeSlice);
		currentProcess->timeRemaining -= timeSlice;
		currentTime += timeSlice;

		if (currentProcess->timeRemaining <= 0) {
			currentProcess->state = TERMINATED;
            deallocateMemory(currentProcess->pid);
			currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
			currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
			completed++;
			printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n", currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
			readyQueueHead = (head + 1) % MAX_PROCESSES;
			total_switches += 1;
		}
		if (currentProcess->timeRemaining > 0) {
			struct PCB* temp = readyQueue[head];
			readyQueue[head] = NULL;
			readyQueue[tail] = temp;
			readyQueueHead = (head + 1) % MAX_PROCESSES;
			readyQueueTail = (tail + 1) % MAX_PROCESSES;
			total_switches += 1;
		}
		checkForNewProcesses(currentTime, false);
	}
}

// -> void
// Purpose: scheduler for FCFS algorithm
void FCFS_Scheduler() {
    int currentTime = 0;
    sortProcessesByArrival();

    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (currentTime < readyQueue[i]->arrivalTime) {
            currentTime = readyQueue[i]->arrivalTime;
        }
        readyQueue[i]->waitingTime = currentTime - readyQueue[i]->arrivalTime;
        contextSwitch(readyQueue[i]);
        executeProcess(readyQueue[i]->burstTime, readyQueue[i]->pid);
        printf("P%d [%d - %d]\n", readyQueue[i]->pid, currentTime, currentTime + readyQueue[i]->burstTime);
        currentTime += readyQueue[i]->burstTime;

        readyQueue[i]->turnTime = currentTime - readyQueue[i]->arrivalTime;

	readyQueue[i]->timeRemaining = 0;
        readyQueue[i]->state = TERMINATED;
        deallocateMemory(readyQueue[i]->pid);
	total_switches += 1;
    }
    printf("\n\n");
}

// int -> void
// Purpose: display the process table values and calculate average waiting and turnaround times.
void display_results(int schedulerIndex) {
    printf("\nProcess Table:\n");
    printf("PID\tArrival\tBurst\tWaiting\tTurnaround\n");

    for (int i = 0; i < MAX_PROCESSES; i++) {
        printf("%d\t%d\t%d\t%d\t%d\n",
            processTable[i].pid,
            processTable[i].arrivalTime,
            processTable[i].burstTime,
            processTable[i].waitingTime,
            processTable[i].turnTime);
    }

    float avgWaitingTime = 0;
    float avgTurnaroundTime = 0;
    for (int i = 0; i < MAX_PROCESSES; i++) {
        avgWaitingTime += processTable[i].waitingTime;
        avgTurnaroundTime += processTable[i].turnTime;
    }
    avgWaitingTime /= MAX_PROCESSES;
    avgTurnaroundTime /= MAX_PROCESSES;

    printf("Average Waiting Time: %.2f\n", avgWaitingTime);
    printf("Average Turnaround Time: %.2f\n", avgTurnaroundTime);

    metrics_table[schedulerIndex].avg_waiting_time = avgWaitingTime;
    metrics_table[schedulerIndex].avg_turn_time = avgTurnaroundTime;

    for (int i = 0; i < MAX_PROCESSES; i++) {
	    printf("Process ID: %d, Process PC: %d, Process ACC: %d, Process State: %d, Process Time: %d, Process Priority: %d, Process Response Ratio: %f\n", processTable[i].pid, processTable[i].PC, processTable[i].ACC, processTable[i].state, processTable[i].timeRemaining, processTable[i].priority, processTable[i].responseRatio);
    }
}

// string array for display purposes
char *string[] = {"FCFS", "Round Robin", "Priority", "SPN", "SRT", "HRRN", "Feedback"};

// -> void
// Purpose: Display the metric data
void display_metrics() {
	for (int i = 0; i < MAX_ALGORITHMS; i++) {
		printf("%s: \n", string[i]);
		printf("\tExecution Time: %ld \n", metrics_table[i].exec_time);
		printf("\tContext Switches: %d \n", metrics_table[i].context_switches);
		printf("\tAverage Waiting Time: %lf \n", metrics_table[i].avg_waiting_time);
		printf("\tAverage Turnaround Time: %lf \n", metrics_table[i].avg_turn_time);
		metrics_table[i].cpu_utilization = 100.0;
		printf("\tCPU Utilization: %lf \n", metrics_table[i].cpu_utilization);
		printf("\tCache Efficiency: %lf \n", metrics_table[i].cacheEfficiency);
	}
}


// void -> void
// Purpose: Initialize the Interrupt Vector Table (IVT) with interrupt types, priorities, and their handlers.
void initIVT() {
    IVT[TIMER_INT] = (struct IVTEntry){1, timerInterrupt};
    IVT[IO_INT] = (struct IVTEntry){2, ioInterrupt};
    IVT[SYSCALL_INT] = (struct IVTEntry){3, syscallInterrupt};
    IVT[TRAP_INT] = (struct IVTEntry){4, trapInterrupt};
}

// -> int
// Purpose: main, initializes the simulator and lets the user select scheduler algorithms
int main(){
    while (1) {
        initIVT();
	    initCache();
	    loadProgram();
	    initMemoryTable();
	    initProcesses();
	    total_switches = 0;
	    cacheHits = 0;
	    cacheMisses = 0;
	    
	    pthread_mutex_init(&memory_mutex, NULL);
	    sem_init(&cache_semaphore, 0, 1);
	    bool quit = false;
	    int scheduler;
	    printf("\n 0: FCFS \n 1: Round Robin \n 2: Priority \n 3: Shortest Process Next (SPN) \n 4: Shortest Remaining Time (SRT) \n 5: Highest Response Ratio Next (HRRN) \n 6: Feedback \n 7: QUIT \n ");
	    printf("Select a scheduling method: ");
	    scanf("%d", &scheduler);
	    clock_t start_time = clock();
	    clock_t end_time;
	    switch(scheduler) {
		    case 0:
			    FCFS_Scheduler();
			    end_time = clock();
			    metrics_table[0].exec_time = end_time - start_time;
			    metrics_table[0].context_switches = total_switches;
			    metrics_table[0].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 1:
			    int timeQuantum;
			    printf("Insert Time Quantum for Round Robin: ");
			    scanf("%d", &timeQuantum);
			    RoundRobin_Scheduler(timeQuantum);
			    end_time = clock();
                            metrics_table[1].exec_time = end_time - start_time;
			    metrics_table[1].context_switches = total_switches;
			    metrics_table[1].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 2:
			    Priority_Scheduler();
			    end_time = clock();
                            metrics_table[2].exec_time = end_time - start_time;
			    metrics_table[2].context_switches = total_switches;
			    metrics_table[2].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 3:
			    SPN_Scheduler();
			    end_time = clock();
                            metrics_table[3].exec_time = end_time - start_time;
			    metrics_table[3].context_switches = total_switches;
			    metrics_table[3].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 4:
			    SRT_Scheduler();
			    end_time = clock();
                            metrics_table[4].exec_time = end_time - start_time;
			    metrics_table[4].context_switches = total_switches;
			    metrics_table[4].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 5:
			    HRRN_Scheduler();
			    end_time = clock();
                            metrics_table[5].exec_time = end_time - start_time;
			    metrics_table[5].context_switches = total_switches;
			    metrics_table[5].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 6:
			    Feedback_Scheduler();
			    end_time = clock();
                            metrics_table[6].exec_time = end_time - start_time;
			    metrics_table[6].context_switches = total_switches;
			    metrics_table[6].cacheEfficiency = (double) cacheHits / (double) (cacheHits + cacheMisses);
			    break;
		    case 7:
			    printf("User is quitting simulation!\n");
			    quit = true;
			    break;
		    default:
			    printf("Invalid scheduler number.\n");
			    break;
	    }
	    if (quit) {
		    break;
	    }
	    display_results(scheduler);
    }
    display_metrics();
    return 0;
}