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

// Lock
pthread_mutex_t cpu_mutex;
pthread_mutex_t memory_mutex;
pthread_mutex_t scheduling_mutex;
pthread_mutex_t interrupts_mutex;

//cpu core registration system.
struct CpuCore {
    int PC;
    int ACC;
    int IR;
    int startMemAdd;
    int endMemAdd;
    int interruptFlag;
    int interruptType;
    int carryFlag;
    int zeroFlag;
    int overFlowFlag;
    int errorFlag;
    int currentProcess;
    int coreID;
};

struct CpuCore cores[2];

void initCpuCores(){
    //core 1 initializes here
    cores[0].coreID = 0;
    cores[0].PC = 0;
    cores[0].ACC = 0;
    cores[0].IR = 0;
    cores[0].startMemAdd = 0;
    cores[0].endMemAdd = 19;
    cores[0].interruptFlag = 0;
    cores[0].interruptType = -1;
    cores[0].carryFlag = 0;
    cores[0].zeroFlag = 0;
    cores[0].overFlowFlag = 0;
    cores[0].errorFlag = 0;
    cores[0].currentProcess = 0;
    
    //core 2 initializes here
    cores[1].coreID = 1;
    cores[1].PC = 20;
    cores[1].ACC = 0;
    cores[1].IR = 0;
    cores[1].startMemAdd = 20;
    cores[1].endMemAdd = 37;
    cores[1].interruptFlag = 0;
    cores[1].interruptType = -1;
    cores[1].carryFlag = 0;
    cores[1].zeroFlag = 0;
    cores[1].overFlowFlag = 0;
    cores[1].errorFlag = 0;
    cores[1].currentProcess = 3;
} 

// Interrupt priority definitions
#define TIMER_INTERRUPT 0
#define IO_INTERRUPT 1
#define SYSCALL_INTERRUPT 2
#define TRAP_INTERRUPT 3

#define MAX_INTERRUPTS 4

// Gian's updated code
// structure for Interrupt Vector Table (IVT)
typedef struct {
    int priority;  // numeric priority of the interrupt
    void (*handler)(int);  // pointer to the interrupt handler function
} IVTEntry;

IVTEntry IVT[MAX_INTERRUPTS];

//Define constants for instruction set
// instruction types
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

// Memory System
//Define cache and RAM sizes
#define L1_CACHE_SIZE 64
#define L2_CACHE_SIZE 128
#define RAM_SIZE 256
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

//Mutexes and semaphores for safe access
sem_t cache_semaphore;

//Structure for the Memory Table entry
struct MemoryBlock {
 int processID;
 int memoryStart;
 int memoryEnd;
 int isFree; //1 = free, 0 = allocated
};
#define MEMORY_TABLE_SIZE 10
struct MemoryBlock memoryTable[RAM_SIZE / 100];

// Process Variables and Structs (M3)
#define MAX_PROCESSES 6
#define TIME_SLICE 5
#define MAX_MESSAGES 10

struct MessageQueue {
	int messages[MAX_MESSAGES];
	int data[MAX_MESSAGES];
	int size;
};

struct PCB {
	int pid;
	int pc;
	int acc;
	int state; // Process state (0 = ready, 1 = running, 3 = terminated)
	int time; // Process time
	int priority; // Process Priority Level (Higher number = higher priority)
	struct MessageQueue messagesQueue; // Message Queue from IPC
};

// Core 1: Processes 1 - 3
// Core 2: Processes 4 - 6
struct PCB processTable[MAX_PROCESSES];

// Messages List
#define START 1
#define NEW_PC 2
#define NEW_ACC 3

// Interrupts (M4)
void (*IVT[3])(int); // Interrupt Vector Table
#define TIMER_INTERRUPT 0
#define IO_INTERRUPT 1
#define SYSTEMCALL_INTERRUPT 2

int generateRandomNum() {
	int randomNum = rand() % 2;
	return randomNum;
}


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
    break;
 }

 }
 if(!successful_alloc){
    printf("Memory Allocation Error: Not enough space for Process %d \n", processID);
 }
    return 0;
}

//function for memory deallocation
void deallocateMemory(int processID) {
 //do: search memoryTable for blocks belonging to processID and free them (isFree = 1)
  bool successful_dealloc = false;
 for (int i = 0; i < RAM_SIZE / 100; i++){
    if (memoryTable[i].processID == processID){
        memoryTable[i].processID = -1;
        memoryTable[i].isFree = 1;
        printf("Memory Deallocation: ProcessID: %d Memory Start: %d Memory End: %d \n", processID, memoryTable[i].memoryStart, memoryTable[i].memoryEnd);
        successful_dealloc = true;
        break;
    }
 }
 if(!successful_dealloc){
    printf("Memory Deallocation Error: ProcessID not found\n");
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

//handle cache lookup
int cacheLookup(int address) {
    //do: Implement L1 and L2 cache lookup logic
    //check L1 cache first; if miss, check L2 cache; if both miss, access RAM
    //return data if found or load from RAM if not found 
    //if in cache 1
    for(int i = 0; i < L1_CACHE_SIZE; i++){
        if(address == L1Tags[i].address){
            printf("L1 Cache hit value: %d \n", L1Cache[i]);
            return L1Cache[i];
        }
    }
    
    //if in cache 2
    for (int i = 0; i < L2_CACHE_SIZE; i++){
        if(address == L2Tags[i].address){
            printf("L2 Cache hit value:%d \n", L2Cache[i]);
            return L2Cache[i];
        }
    }

    // so that the cache updates cache and RAM address
    int value = RAM[address];
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

//add more based on the rquirements
//Function to initialize memory with sample instructions
void loadProgram() {
    // Complete: Load sample instructions into RAM
    // Eg:
    // RAM[0] = LOAD; RAM[1] = 10; // LOAD 10 into ACC
    // Add more instructions as needed

    // Core 1 instructions
    RAM[0] = LOAD;  RAM[1] = 10;    // Load 10 into ACC (for Core 0)
    RAM[2] = ADD;   RAM[3] = 5;     // Add 5 to ACC (Core 0 ACC = 15)
    RAM[4] = STORE; RAM[5] = 3;     // Store ACC value (15) to RAM address 20
    RAM[6] = LOAD;  RAM[7] = 12;    // Load 12 into ACC
    RAM[8] = SUB;   RAM[9] = 3;     // Subtract 3 from ACC (Core 0 ACC = 9)
    RAM[10] = JZ;   RAM[11] = 16;   // Jump to address 16 if zeroFlag is set
    RAM[12] = JMP;  RAM[13] = 16;   // Unconditional jump to address 16
    RAM[14] = MUL;  RAM[15] = 2;    // Multiply ACC by 2 (Should be skipped if JZ succeeds)
    RAM[16] = LOAD; RAM[17] = 2;    // Loads 2 into ACC
    RAM[18] = DIV;  RAM[19] = 2;    // Divide ACC by 2 = 1

    // Core 2 instructions
    RAM[20] = LOAD;  RAM[21] = 3;     // Core 1 loads 3 into ACC
    RAM[22] = ADD;   RAM[23] = 10;    // Adds 10 to ACC (Core 1 ACC = 13)
    RAM[24] = AND;   RAM[25] = 4;     // Perform AND with 4 (binary operation on ACC)
    RAM[26] = OR;    RAM[27] = 8;     // Perform OR with 8 (Core 1 ACC modified)
    RAM[28] = STORE; RAM[29] = 25;    // Store ACC to RAM address 25
    RAM[30] = LOAD;  RAM[31] = 7;     // Load 7 into ACC
    RAM[32] = JMP;   RAM[33] = 36;    // Jump to address 22
    RAM[34] = DIV;   RAM[35] = 2;     // Divide ACC by 2 (should be skipped if JMP succeeds)
    RAM[36] = SUB;   RAM[37] = 1;     // Subtract 1 from ACC (Core 1 ACC = 6 after all operations)
}

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
//Function to simulate instruction fetching
void fetch(int coreNum) {
 cores[coreNum].IR = accessMemory(cores[coreNum].PC, 1, false); //Fetch the next instruction
}

void execute(int coreNum){

    //Decode the instruction in IR
    //Implement logic to extract the opcode and operands   
	struct PCB currProcess = processTable[cores[coreNum].currentProcess];
	int opcode = cores[coreNum].IR;
	int operand = accessMemory(cores[coreNum].PC + 1, 1, false);
	int result;
	int prev;
	int core = coreNum + 1;
	if (currProcess.state == 3 && currProcess.time <= 0) {
		printf("terminated process detected in execute.\n");
		return;
	}
	
	switch (opcode) {
        case ADD: 
            prev = cores[coreNum].ACC;
            result = cores[coreNum].ACC + operand;
            cores[coreNum].ACC = result;
            printf("Core %d Operation Adding: %d + %d = %d\n", core, prev, operand, cores[coreNum].ACC);
            break;
        case SUB:
            prev = cores[coreNum].ACC;
            result = cores[coreNum].ACC - operand;
            cores[coreNum].ACC = result;
            printf("Core %d Operation: Subtracting: %d - %d = %d\n", core, prev, operand, cores[coreNum].ACC);
            break;
        case MUL:
            prev = cores[coreNum].ACC;
            result = cores[coreNum].ACC * operand;
            cores[coreNum].ACC = result;
            printf("Core %d Operation Multiplying: %d * %d = %d\n", core, prev, operand, cores[coreNum].ACC);
            break;
        case DIV:
            if(operand != 0){
                prev = cores[coreNum].ACC;
                result = cores[coreNum].ACC / operand;
                cores[coreNum].ACC = result;
                printf("Core %d Operation: Dividing: %d / %d = %d\n", core, prev, operand, cores[coreNum].ACC);
                break;
            }
            else{
                printf("Core %d Operation: Division Error: Division by zero\n", core);
                cores[coreNum].errorFlag = 1;
                break;
            }
        case LOAD:
            prev = cores[coreNum].ACC;
            cores[coreNum].ACC = operand;
			printf("Core %d Operation: Loading data in ACC: %d -> %d\n",core , prev, cores[coreNum].ACC);
			break;
        case STORE:
            accessMemory(operand, cores[coreNum].ACC, true);
            printf("Core %d Operation: Storing data into memory from ACC to Adress: %d -> %d\n", core, cores[coreNum].ACC, operand);
			break;
        case AND:
            prev = cores[coreNum].ACC;
            result = cores[coreNum].ACC & operand;
            cores[coreNum].ACC = result;
			printf("Core %d Operation: AND operation: %d & %d = %d \n", core,  prev, operand, cores[coreNum].ACC);
            break;
        case OR:
            prev = cores[coreNum].ACC;
            result = cores[coreNum].ACC | operand;
            cores[coreNum].ACC = result;
            printf("Core %d Operation: OR operation: %d | %d = %d \n", core , prev, operand, cores[coreNum].ACC);
            break;
        case JMP:
            if (operand >= cores[coreNum].startMemAdd && operand < cores[coreNum].endMemAdd){
                prev = cores[coreNum].PC;
                cores[coreNum].PC = operand - 2;
                printf("Core %d Operation: Jump: PC has been changed from %d to %d\n",core ,  prev, cores[coreNum].PC);
                break;
            }
            else{
                printf("Core %d Operation: Error: Invalid Jump attempt. Out of Bounds Error\n", core);
                cores[coreNum].errorFlag = 1;
                break;
            }
        case JZ:
            if(cores[coreNum].zeroFlag == 1){
                if(operand >= cores[coreNum].startMemAdd && operand < cores[coreNum].endMemAdd){
                    prev = cores[coreNum].PC;
                    cores[coreNum].PC = operand - 2;
                    printf("Core %d Operation: Jump Zero: PC has been changed from %d to %d\n", core, prev, cores[coreNum].PC);
                    break;
                    }
                    else{
                        printf("Core %d Operation: Error: Invalid Jump attempt. Out of Bounds Error\n",core);
                        cores[coreNum].errorFlag = 1;
                        break;
                    }
                
            }
            else{
                printf("Core %d Operation: Jump Zero: Unsuccessful jump, no zero flag present.\n", core);
                break;
            }
		default:
			// Handle undefined/invalid opcodes
			printf("Core %d Operation: Invalid opcode given. Please try again.\n", core);
            cores[coreNum].errorFlag = 1;
			break;
        }
    cores[coreNum].PC += 2; //Move to the next instruction
}

bool completeProcessesForAll() {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		if (processTable[i].state != 3) {
			return false;
		}
	}
	return true;
}

bool complete_processes(int coreID) {
	int startIndex;
	if (coreID == 0) {
		startIndex = 0;
	} else {
		startIndex = 3;
	}
	int endIndex = startIndex + 2;
	for (int i = startIndex; i < endIndex; i++) {
		if (processTable[i].state != 3) {
			return false;
		}
	}
	return true;
}

//Function to simulate the instruction cycle with concurrency
void* cpuCore(void* arg) {
	int coreNum = *(int *)arg;
	while (1) {
		if (cores[coreNum].errorFlag == 1) {
			break;
		}
	if (complete_processes(coreNum)) {
		printf("Processes completed in cpuCore!\n");
		break;
	}
	if (cores[coreNum].interruptFlag) {
		sleep(2);
	}
        fetch(coreNum); //fetch instruction
        execute(coreNum); //decode and execute instruction
	processTable[cores[coreNum].currentProcess].pc = cores[coreNum].PC;
	processTable[cores[coreNum].currentProcess].acc = cores[coreNum].ACC;
	processTable[cores[coreNum].currentProcess].time -= TIME_SLICE;
	if (processTable[cores[coreNum].currentProcess].time <= 0) {
		processTable[cores[coreNum].currentProcess].time = 0;
		processTable[cores[coreNum].currentProcess].state = 3;
	} else {
		processTable[cores[coreNum].currentProcess].state = 0;
	}
        //Stop condition (example: check PC bounds)
        if (cores[coreNum].PC >= cores[coreNum].endMemAdd || cores[coreNum].PC < cores[coreNum].startMemAdd) {
		printf("Core %d PC: %d, endMem: %d, startMem: %d\n", coreNum + 1, cores[coreNum].PC, cores[coreNum].endMemAdd, cores[coreNum].startMemAdd);
		break;
	}
	sleep(1);
    }
    if(cores[coreNum].errorFlag == 1){
    	printf("Core %d Error Exit\n", coreNum +1);
    }
    else{
    	printf("Core %d Execution Complete\n", coreNum + 1);
    }
        return NULL;
}

void initProcesses() {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		processTable[i].pid = 100 + (i * 100);
		if (i < 3) {
			processTable[i].pc = 0;
		} else {
			processTable[i].pc = 20;
		}
		processTable[i].acc = 0;
		processTable[i].state = 0;
		processTable[i].time = (rand() % 14) + 1;
		processTable[i].priority = (rand() % 5) + 1;
		processTable[i].messagesQueue.size = 0;
	}
}

void contextSwitch(int currProcess, int nextProcess, int coreID) {
	if (processTable[nextProcess].state != 0 && processTable[nextProcess].state != 1) {
		printf("Invalid context switch to non-ready/running process.\n");
		return;
	}
	processTable[currProcess].pc = cores[coreID].PC;
	processTable[currProcess].acc = cores[coreID].ACC;
	if (processTable[currProcess].state != 3) {
		processTable[currProcess].state = 0;
	}
	cores[coreID].PC = processTable[nextProcess].pc;
	cores[coreID].ACC = processTable[nextProcess].acc;
	processTable[nextProcess].state = 1;

	printf("Context switch complete! Process %d to Process %d\n", processTable[currProcess].pid, processTable[nextProcess].pid);
}

void allProcesses(int coreID) {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		printf("Process ID: %d, PC: %d, ACC: %d, State: %d, Time: %d\n", processTable[i].pid, processTable[i].pc, processTable[i].acc, processTable[i].state, processTable[i].time);
	}
	return;
}

void round_robin(int coreID) {
	int currentProcess = cores[coreID].currentProcess;
	int nextProcess;
	if (coreID == 0) {
		printf("SHOULD BE 1 Core %d in round robin\n", coreID + 1); 
		nextProcess = (currentProcess + 1) % 3;
	} else {
		printf("SHOULD BE 2 Core %d in round robin\n", coreID + 1);
		nextProcess = ((currentProcess + 1) % 3) + 3;
	}
	allProcesses(coreID);
	while (processTable[nextProcess].state == 3) {
		if (coreID == 0) {
			printf("SHOULD BE 1 Core %d in round robin\n", coreID + 1);
			nextProcess = (nextProcess + 1) % 3;
		} else {
			printf("SHOULD BE 2 Core %d in round robin\n", coreID + 1); 
			nextProcess = ((nextProcess + 1) % 3) + 3;
		}
		if (nextProcess == currentProcess) {
			if (processTable[nextProcess].state != 3) {
				contextSwitch(currentProcess, nextProcess, coreID);
				cores[coreID].currentProcess = nextProcess;
			} else {
				printf("All processes complete for core: %d\n", coreID + 1);
				break;
			}
		}
	}
	if (processTable[nextProcess].state == 0) {
		contextSwitch(currentProcess, nextProcess, coreID);
		cores[coreID].currentProcess = nextProcess;
	}
	return;
}

void priorityScheduler(int coreID) {
	int highestIndex = -1;
	int highestPriority = -1;
	int startingIndex;
	
	if (coreID = 0) {
		startingIndex = 0;
	} else {
		startingIndex = 3;
	}

	for (startingIndex; startingIndex < startingIndex + 3; startingIndex++) {
		if (processTable[startingIndex].state == 0) {
			if (processTable[startingIndex].priority > highestPriority) {
				highestPriority = processTable[startingIndex].priority;
				highestIndex = startingIndex;
			}
		}
	}
	if (highestIndex != -1) {
		contextSwitch(cores[coreID].currentProcess, highestIndex, coreID);
		cores[coreID].currentProcess = highestIndex;
	} else {
		printf("No ready processes found.\n");
	}
}
// int int int -> void
// Purpose: Takes a processID (receiver), a message, and data for that message and sends the message and data to the receiver.
void sendMessage(int process, int message, int dataSent, int coreID) {
	struct MessageQueue *queue = &processTable[process].messagesQueue;

	if (queue->size < MAX_MESSAGES) {
		int index = queue->size;
		queue->messages[index] = message;
		queue->data[index] = dataSent;
		queue->size++;
		printf("Message sent from Process %d to Process %d\n", cores[coreID].currentProcess, process);
	} else {
		printf("Message Queue is full for Process %d!\n", process);
	}
}

void recieveMessage(int process) {
	struct MessageQueue *queue = &processTable[process].messagesQueue;
	if (queue->size == 0) {
		printf("No messages found for Process: %d\n", process);
	}
	int message = queue->messages[0];
	int data = queue->data[0];

	switch (message) {
		case START:
			printf("Process %d received a START message.\n", process);
			processTable[process].state = 1;
			break;
		case NEW_PC:
			printf("Process %d received a NEW_PC message.\n", process);
			processTable[process].pc = data;
			break;
		case NEW_ACC:
			printf("Process %d received a NEW_ACC message.\n", process);
			processTable[process].acc = data;
			break;
		default:
			printf("Process %d received an INVALID message: %d.\n", process, message);
	}
	for (int i = 0; i < queue->size; i++) {
		queue->messages[i] = queue->messages[i + 1];
		queue->data[i] = queue->data[i + 1];
	}
	queue->size--;
}

void* schedulingTask(void* arg) {
	int coreNum = *(int *)arg;
	while (1) {
		if (cores[coreNum].interruptFlag) {
			sleep(2);
		}
		pthread_mutex_lock(&scheduling_mutex);
		round_robin(coreNum);
		pthread_mutex_unlock(&scheduling_mutex);
		if (complete_processes(coreNum)) {
			printf("Completion detected in schedulingTask!\n");
			break;
		}
		sleep(1);
	}
}

// Gian's updated code
void timerInterrupt(int coreID) {
    printf("Timer Interrupt Detected in Core %d!\n", coreID + 1);
    sleep(1);
    printf("Timer Done in Core %d!\n", coreID + 1);
}

void ioInterrupt(int coreID) {
    printf("IO Interrupt Detected in Core %d!\n", coreID + 1);
    sleep(1);
    printf("IO Done in Core %d!\n", coreID + 1);
}

void syscallInterrupt(int coreID) {
    printf("System Call Interrupt Detected in Core %d!\n", coreID + 1);
    sleep(1);
    printf("System Call Done in Core %d!\n", coreID + 1);
}

void trapInterrupt(int coreID) {
    printf("Trap Interrupt Detected in Core %d!\n", coreID + 1);
    sleep(1);
    printf("Trap Handling Done in Core %d!\n", coreID + 1);
}

// Gian's updated code
void initIVT() {
    IVT[TIMER_INTERRUPT] = (IVTEntry){1, timerInterrupt};
    IVT[IO_INTERRUPT] = (IVTEntry){2, ioInterrupt};
    IVT[SYSCALL_INTERRUPT] = (IVTEntry){3, syscallInterrupt};
    IVT[TRAP_INTERRUPT] = (IVTEntry){4, trapInterrupt};
}

// Gian's updated code
// ISR with prioritization
void ISR(int coreID, int interruptType) {
    for (int i = 0; i < MAX_INTERRUPTS; i++) {
        if (interruptType == i) {
            printf("Handling Interrupt Type %d with Priority %d on Core %d\n", i, IVT[i].priority, coreID + 1);
            IVT[i].handler(coreID);
            break;
        }
    }
}

// check for pending interrupts and handle the highest-priority one
void checkAndHandleInterrupts(int coreID) {
    static int interruptQueue[MAX_INTERRUPTS] = {0};
    static int queueSize = 0;

    if (queueSize > 0) {
        int highestPriorityIndex = 0;
        for (int i = 1; i < queueSize; i++) {
            if (IVT[interruptQueue[i]].priority < IVT[interruptQueue[highestPriorityIndex]].priority) {
                highestPriorityIndex = i;
            }
        }

        int interruptType = interruptQueue[highestPriorityIndex];

        // handle the interrupt
        ISR(coreID, interruptType);

        // remove the handled interrupt from the queue
        for (int i = highestPriorityIndex; i < queueSize - 1; i++) {
            interruptQueue[i] = interruptQueue[i + 1];
        }
        queueSize--;
    }
}

// Gian's updated code
void* generateInterrupts(void* arg) {
    int coreID = *(int*)arg;
    while (1) {
        sleep(rand() % 3 + 1);
        int interruptType = rand() % MAX_INTERRUPTS;
        printf("Generated Interrupt Type %d for Core %d\n", interruptType, coreID + 1);
        ISR(coreID, interruptType);
    }
}

void timerStart() {
	int randomNum = generateRandomNum();
	if (randomNum == 0) {
		cores[0].interruptType = TIMER_INTERRUPT;
		ISR(0, TIMER_INTERRUPT);
	} else {
		cores[1].interruptType = TIMER_INTERRUPT;
		ISR(1, TIMER_INTERRUPT);
	}
}

void* interruptTask(void* arg) {
	signal(SIGALRM, timerStart);
	while (1) {
		if (completeProcessesForAll()) {
			printf("Completion detected in interrupt thread!\n");
			break;
		}
		pthread_mutex_lock(&interrupts_mutex);
		char input = getchar();
		int coreNum;
		if (input == 'k') {
			coreNum = generateRandomNum();
			cores[coreNum].interruptType = IO_INTERRUPT;
			ISR(coreNum, cores[coreNum].interruptType);
		} else if (input == 's') {
			coreNum = generateRandomNum();
			cores[coreNum].interruptType = SYSTEMCALL_INTERRUPT;
			ISR(coreNum, cores[coreNum].interruptType);
		} else {
			alarm(1);
		}
		pthread_mutex_unlock(&interrupts_mutex);
	}
	printf("Interrupt Thread completed!\n");
}


void startCores(){
 initCpuCores();
 int *core1 = malloc(sizeof(int));
 *core1 = 0;

 int *core2 = malloc(sizeof(int));
 *core2 = 1;

 //create a thread for concurrent processing
 pthread_t cpu_thread;
 pthread_t cpu_thread2;
 pthread_t schedulingthread;
 pthread_t schedulingthread2;
 pthread_t interruptthread;
 
 pthread_create(&cpu_thread, NULL, cpuCore, core1);
 pthread_create(&cpu_thread2, NULL, cpuCore, core2);

 pthread_create(&schedulingthread, NULL, schedulingTask, core1);
 pthread_create(&schedulingthread2, NULL, schedulingTask, core2);
 
 pthread_create(&interruptthread, NULL, interruptTask, NULL);

 //wait for the thread to complete
 pthread_join(cpu_thread, NULL);
 printf("CPU 1 thread joined.\n");
 pthread_join(cpu_thread2, NULL);
 printf("CPU 2 thread joined.\n");
 pthread_join(schedulingthread, NULL);
 printf("scheduling thread 1 joined.\n");
 pthread_join(schedulingthread2, NULL);
 printf("scheduling thread 2 joined.\n");
 pthread_join(interruptthread, NULL);
 printf("interrupt thread joined.\n");

 free(core1);
 free(core2);
 printf("All threads terminated!\n");
}

int main() {
    loadProgram();  // initialize memory with instructions
    initMemoryTable();
    initCache();
    initProcesses();
    initIVT();  // initialize Interrupt Vector Table

    pthread_mutex_init(&memory_mutex, NULL);
    sem_init(&cache_semaphore, 0, 1);
    pthread_mutex_init(&scheduling_mutex, NULL);
    pthread_mutex_init(&interrupts_mutex, NULL);

    int core0 = 0, core1 = 1;
    pthread_t interruptThread1, interruptThread2;

    // create threads to simulate interrupt generation
    pthread_create(&interruptThread1, NULL, generateInterrupts, &core0);
    pthread_create(&interruptThread2, NULL, generateInterrupts, &core1);

    // join threads
    pthread_join(interruptThread1, NULL);
    pthread_join(interruptThread2, NULL);

    pthread_mutex_destroy(&scheduling_mutex);
    pthread_mutex_destroy(&interrupts_mutex);
    pthread_mutex_destroy(&memory_mutex);
    sem_destroy(&cache_semaphore);

    return 0;
}
