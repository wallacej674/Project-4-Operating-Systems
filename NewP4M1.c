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
#define RAM_SIZE 1024

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

int getIndex(int processID) {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		if (processTable[i].pid == processID) {
			return i;
		}
	}
}

void execute(int processID){
    int prev;
    int result;
    int processNum = getIndex(processID);

    //the fetch
    int instruction = accessMemory(processTable[processNum].PC, 1, false);

    //the decode
    int opcode = instruction;
    int operand = accessMemory(processTable[processNum].PC + 1, 1, false);

    //everything else below is the execute
    switch (opcode) {
        case ADD: 
            result = processTable[processNum].ACC + operand;
            prev = processTable[processNum].ACC;
            processTable[processNum].ACC = result;
            printf("PID %d: Operation Add, %d + %d = %d\n", processTable[processNum].pid, prev, operand, processTable[processNum].ACC);
            processTable[processNum].PC += 2; //Move to the next instruction
            break;
        case SUB:
            result  = processTable[processNum].ACC - operand;
            prev = processTable[processNum].ACC;
            processTable[processNum].ACC = result;
            processTable[processNum].PC += 2;
            printf("PID %d Operation: Subtracting: %d - %d = %d\n", processTable[processNum].pid, prev, operand, processTable[processNum].ACC);
            processTable[processNum].PC += 2; //Move to the next instruction
            break;
        case MUL:
            prev = processTable[processNum].ACC;
            result = processTable[processNum].ACC * operand;
            processTable[processNum].ACC = result;
            printf("Core %d Operation Multiplying: %d * %d = %d\n", processTable[processNum].pid, prev, operand, processTable[processNum].ACC);
            processTable[processNum].PC += 2; //Move to the next instruction

            break;
        case DIV:
            if(operand != 0){
                prev = processTable[processNum].ACC;
                result = processTable[processNum].ACC / operand;
                processTable[processNum].ACC = result;
                printf("Core %d Operation: Dividing: %d / %d = %d\n", processTable[processNum].pid, prev, operand, processTable[processNum].ACC);
                processTable[processNum].PC += 2; //Move to the next instruction
                break;
            }
            else{
                printf("pid %d Operation: Division Error: Division by zero\n", processTable[processNum].pid);
                errorFlag = FLAGGED;
                break;
            }
        case LOAD:
            prev = processTable[processNum].ACC;
            processTable[processNum].ACC = operand;
			printf("pid %d Operation: Loading data in ACC: %d -> %d\n",processTable[processNum].pid , prev, processTable[processNum].ACC);
            processTable[processNum].PC += 2; //Move to the next instruction
			break;
        case STORE:
            accessMemory(operand, processTable[processNum].ACC, true);
            printf("pid %d Operation: Storing data into memory from ACC to Adress: %d -> %d\n", processTable[processNum].pid, processTable[processNum].ACC, operand);
            processTable[processNum].PC += 2; //Move to the next instruction
			break;
        case AND:
            prev = processTable[processNum].ACC;
            result = processTable[processNum].ACC & operand;
            processTable[processNum].ACC = result;
			printf("pid %d Operation: AND operation: %d & %d = %d \n", processTable[processNum].pid,  prev, operand, processTable[processNum].ACC);
            processTable[processNum].PC += 2; //Move to the next instruction
            break;
        case OR:
            prev = processTable[processNum].ACC;
            result = processTable[processNum].ACC | operand;
            processTable[processNum].ACC = result;
            printf("pid %d Operation: OR operation: %d | %d = %d \n", processTable[processNum].pid , prev, operand, processTable[processNum].ACC);
            processTable[processNum].PC += 2; //Move to the next instruction
            break;
        case JMP:
            if (operand >= 0  && operand < RAM_SIZE){
                prev = processTable[processNum].PC;
                processTable[processNum].PC = operand - 2;
                printf("pid %d Operation: Jump: PC has been changed from %d to %d\n",processTable[processNum].pid ,  prev, processTable[processNum].PC);
                processTable[processNum].PC += 2; //Move to the next instruction
                break;
            }
            else{
                printf("pid %d Operation: Error: Invalid Jump attempt. Out of Bounds Error\n", processTable[processNum].pid);
                errorFlag = FLAGGED;
                break;
            }
        case JZ:
            if(zeroFlag == 1){
                if(operand >= 0  && operand < RAM_SIZE){
                    prev = processTable[processNum].PC;
                    processTable[processNum].PC = operand - 2;
                    printf("pid %d Operation: Jump Zero: PC has been changed from %d to %d\n", processTable[processNum].pid, prev, processTable[processNum].PC);
                    processTable[processNum].PC += 2; //Move to the next instruction
                    break;
                    }
                    else{
                        printf("pid %d Operation: Error: Invalid Jump attempt. Out of Bounds Error\n",processTable[processNum].pid);
                        errorFlag = FLAGGED;
                        break;
                    }
                
            }
            else{
                printf("pid %d Operation: Jump Zero: Unsuccessful jump, no zero flag present.\n", processTable[processNum].pid);
                break;
            }
		default:
			// Handle undefined/invalid opcodes
			printf("pid: %d Operation: Invalid opcode given. Please try again.\n", processTable[processNum].pid);
            errorFlag = 1;
			break;
        }
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

void loadProgram() {
    // Complete: Load sample instructions into RAM
    // Eg:
    // RAM[0] = LOAD; RAM[1] = 10; // LOAD 10 into ACC
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
    RAM[54] = SUB;   RAM[55] = 75;
    RAM[56] = OR;    RAM[57] = 85;
    RAM[58] = LOAD;  RAM[59] = 10;
    RAM[60] = ADD;   RAM[61] = 40;
    RAM[62] = SUB;   RAM[63] = 30;
    RAM[64] = MUL;   RAM[65] = 40;
    RAM[66] = DIV;   RAM[67] = 50;
    RAM[68] = STORE; RAM[69] = 60;
    RAM[70] = JMP;   RAM[71] = 72;
    RAM[72] = JZ;    RAM[73] = 80;
    RAM[74] = AND;   RAM[75] = 90;
    RAM[76] = OR;    RAM[77] = 100;
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

void initProcesses(){
	int arrivalTimeRange;
	int burstTimeRange;
	printf("Set a max for arrival time range: ");
	scanf("%d", &arrivalTimeRange);
	printf("Set a max for burst time range: ");
	scanf("%d", &burstTimeRange);
	for(int i = 0; i < MAX_PROCESSES; i++){
		processTable[i].pid = i + 100;
		processTable[i].PC = 0;
		processTable[i].ACC = 0;
		processTable[i].state = READY;
		processTable[i].priority = (rand() % 6) + 1;
		processTable[i].arrivalTime = (rand() % arrivalTimeRange) + 1;
		processTable[i].burstTime = (rand() % burstTimeRange) + 1;
		processTable[i].waitingTime = 0;
		processTable[i].turnTime = 0;
		processTable[i].timeRemaining = processTable[i].burstTime;
		processTable[i].responseRatio = 0;
		processTable[i].inQueue = false;
	}
}

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

void checkForNewProcesses(int time) {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		struct PCB* process = &processTable[i];
		if (process->inQueue) {
			continue;
		}
		if (processTable[i].arrivalTime <= time) {
			readyQueue[readyQueueTail] = process;
			readyQueueSize += 1;
			process->inQueue = true;
			readyQueueTail = (readyQueueTail + 1) % MAX_PROCESSES;
		}
	}
}

void sortByPriority() {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		readyQueue[i] = &processTable[i];
		processTable[i].inQueue = true;
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

void pickPriorityProcess(int currentTime) {
	if (currentTime == 0) {
		return;
	}
	for (int i = 0; i < MAX_PROCESSES; i++) {
		if (readyQueue[i]->arrivalTime <= currentTime && (readyQueue[i]->state != TERMINATED && readyQueue[i]->state != BLOCKED)) {
				readyQueueHead = i;
				return;
		}
	}
}

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

        execute(currentProcess->pid);
	printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + currentProcess->burstTime);
        currentTime += currentProcess->burstTime;

        currentProcess->timeRemaining = 0;
        currentProcess->state = TERMINATED;
        currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
        currentProcess->waitingTime = currentProcess->turnTime - currentProcess->burstTime;
        completed++;

        printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n",
               currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
    }
}
	


void Priority_Scheduler() {
    int currentTime = 0;
    int completed = 0;
    firstProcessInQueue();
    while (completed < MAX_PROCESSES) {
	    printf("%d\n", currentTime);
	    sortByPriority();
	    pickPriorityProcess(currentTime);
        struct PCB* currentProcess = readyQueue[readyQueueHead];

        if (currentTime < currentProcess->arrivalTime) {
            currentTime = currentProcess->arrivalTime;
            continue;
        }

	execute(currentProcess->pid);
	printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + currentProcess->burstTime);
        currentProcess->timeRemaining -= 1;
        currentTime += 1;

        currentProcess->waitingTime = currentTime - currentProcess->arrivalTime - (currentProcess->burstTime - currentProcess->timeRemaining);
        if (currentProcess->timeRemaining <= 0) {
		currentProcess->state = TERMINATED;
            currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
            completed++;
            printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n",
                currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
            readyQueueHead = (readyQueueHead + 1) % MAX_PROCESSES;
        }
	checkForNewProcesses(currentTime);
    }
}


void RoundRobin_Scheduler(int timeQuantum) {
	int currentTime = 0;
	int completed = 0;
	firstProcessInQueue();
	while (completed < MAX_PROCESSES) {
		int head = readyQueueHead;
		int tail = readyQueueTail;
		struct PCB* currentProcess = readyQueue[head];
		if (currentTime < currentProcess->arrivalTime) {
			currentTime = currentProcess->arrivalTime;
		}
		execute(currentProcess->pid);
		printf("P%d [%d - %d]\n", currentProcess->pid, currentTime, currentTime + currentProcess->burstTime);		
		int timeSlice = (currentProcess->timeRemaining < timeQuantum) ? currentProcess->timeRemaining : timeQuantum;
		currentProcess->timeRemaining -= timeSlice;
		currentTime += timeSlice;
		currentProcess->waitingTime = currentTime - currentProcess->arrivalTime - (currentProcess->burstTime - currentProcess->timeRemaining);

		if (currentProcess->timeRemaining <= 0) {
			currentProcess->state = TERMINATED;
			currentProcess->turnTime = currentTime - currentProcess->arrivalTime;
			completed++;
			printf("Process %d completed. Waiting Time: %d, Turnaround Time: %d\n", currentProcess->pid, currentProcess->waitingTime, currentProcess->turnTime);
			readyQueueHead = (head + 1) % MAX_PROCESSES;
		}
		if (currentProcess->timeRemaining > 0) {
			struct PCB* temp = readyQueue[head];
			readyQueue[head] = NULL;
			readyQueue[tail] = temp;
			readyQueueHead = (head + 1) % MAX_PROCESSES;
			readyQueueTail = (tail + 1) % MAX_PROCESSES;
		}
		checkForNewProcesses(currentTime);
	}
}


void FCFS_Scheduler() {
    int currentTime = 0;
    sortProcessesByArrival();

    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (currentTime < readyQueue[i]->arrivalTime) {
            currentTime = readyQueue[i]->arrivalTime;
        }

        readyQueue[i]->state = RUNNING;
        readyQueue[i]->waitingTime = currentTime - readyQueue[i]->arrivalTime;

	execute(readyQueue[i]->pid);
        printf("P%d [%d - %d]\n", readyQueue[i]->pid, currentTime, currentTime + readyQueue[i]->burstTime);
        currentTime += readyQueue[i]->burstTime;

        readyQueue[i]->turnTime = currentTime - readyQueue[i]->arrivalTime;

	readyQueue[i]->timeRemaining = 0;
        readyQueue[i]->state = TERMINATED;
    }
    printf("\n\n");
}

// Function to display the process table after scheduling
void display_results() {
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

    // Display PCBs
    for (int i = 0; i < MAX_PROCESSES; i++) {
	    printf("Process ID: %d, Process PC: %d, Process ACC: %d, Process State: %d, Process Time: %d, Process Priority: %d\n", processTable[i].pid, processTable[i].PC, processTable[i].ACC, processTable[i].state, processTable[i].timeRemaining, processTable[i].priority);
    }
}

bool complete_processes() {
	for (int i = 0; i < MAX_PROCESSES; i++) {
		if (processTable[i].state != 3) {
			return false;
		}
	}
	return true;
}

int main(){
    //this will do all of the load programming and other operations.
    initCache();
    loadProgram();
    initMemoryTable();
    initProcesses();

    pthread_mutex_init(&memory_mutex, NULL);
    sem_init(&cache_semaphore, 0, 1);
    
    while (1) {
	    if (complete_processes()) {
		    printf("All processes complete!\n");
		    break;
	    }
	    if (interruptFlag) {
		    printf("Interrupt detected in main loop.\n");
		    while (interruptFlag) {
		    }
	    }
	    // Scheduler Handling
	    int scheduler;
	    printf("\n 0: FCFS \n 1: Round Robin \n 2: Priority \n 3: Shortest Process Next (SPN) \n");
	    printf("Select a scheduling method: ");
	    scanf("%d", &scheduler);
	    switch(scheduler) {
		    case 0:
			    FCFS_Scheduler();
			    break;
		    case 1:
			    int timeQuantum;
			    printf("Insert Time Quantum for Round Robin: ");
			    scanf("%d", &timeQuantum);
			    RoundRobin_Scheduler(timeQuantum);
			    break;
		    case 2:
			    Priority_Scheduler();
			    break;
		    case 3:
			    SPN_Scheduler();
			    break;
		    default:
			    printf("Invalid scheduler number.\n");
			    break;
	    }
    }

    display_results();

    return 0;
}
