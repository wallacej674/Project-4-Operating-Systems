//this is a starter file

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
#define RAM_SIZE 1000
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

// struct for message queues
#define MAX_PROCESSES 6
#define TIME_SLICE 5
#define MAX_MESSAGES 10

struct MessageQueue {
	int messages[MAX_MESSAGES];
	int data[MAX_MESSAGES];
	int size;
};

//the struct to make for future processes.
struct PCB {
	int pid;
	int pc;
	int acc;
	int state; // Process state (0 = ready, 1 = running, 3 = terminated)
	int time; // Process time
	int priority; // Process Priority Level (Higher number = higher priority)
	float responseRatio;
	struct MessageQueue messagesQueue; // Message Queue from IPC
};

//cpu core registration system.
struct CpuCore{
    int PC;
    int ACC;
    int IR;
    int startMemAdd;
    int endMemAdd;
    int interruptFlag; // flag to signal the interrupt
    int carryFlag;
    int zeroFlag;
    int overFlowFlag;
    int errorFlag;
};
struct CpuCore cores[2];

void initCpuCores(){
    //core 1 initializes here
    cores[0].PC = 0;
    cores[0].ACC = 0;
    cores[0].IR = 0;
    cores[0].startMemAdd = 0;
    cores[0].endMemAdd = 19;
    cores[0].interruptFlag = 0;
    cores[0].carryFlag = 0;
    cores[0].zeroFlag = 0;
    cores[0].overFlowFlag = 0;
    cores[0].errorFlag = 0;
    
    //core 2 initializes here
    cores[1].PC = 20;
    cores[1].ACC = 0;
    cores[1].IR = 0;
    cores[1].startMemAdd = 20;
    cores[1].endMemAdd = 37;
    cores[1].interruptFlag = 0;
    cores[1].carryFlag = 0;
    cores[1].zeroFlag = 0;
    cores[1].overFlowFlag = 0;
    cores[1].errorFlag = 0;
} 


//Mutex for safe resource access
pthread_mutex_t memory_mutex;
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
	//struct PCB currProcess = processTable[cores[coreNum].currentProcess];
	int opcode = cores[coreNum].IR;
	int operand = accessMemory(cores[coreNum].PC + 1, 1, false);
	int result;
	int prev;
	int core = coreNum + 1;
	/*if (currProcess.state == 3 && currProcess.time <= 0) {
		printf("terminated process detected in execute.\n");
		return;
	}*/
	
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

//Function to simulate the instruction cycle with concurrency
void* cpuCore(void* arg) {
int coreNum = *(int *)arg;
    while (1) {
        if(cores[coreNum].errorFlag == 1){
        break;
        }
        //Lock memory for exclusive access
        fetch(coreNum); //fetch instruction
        execute(coreNum); //decode and execute instruction
        //Stop condition (example: check PC bounds)
        if (cores[coreNum].PC >= cores[coreNum].endMemAdd || cores[coreNum].PC < cores[coreNum].startMemAdd) break;
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


void startCores(){
 initCpuCores();

 int *core1 = malloc(sizeof(int));
 *core1 = 0;

 int *core2 = malloc(sizeof(int));
 *core2 = 1;

 //create a thread for concurrent processing
 pthread_t cpu_thread;
 pthread_t cpu_thread2;
 pthread_t producer_thread, consumer_thread;
 
 pthread_create(&cpu_thread, NULL, cpuCore, core1);
 pthread_create(&cpu_thread2,NULL, cpuCore, core2);
 pthread_create(&producer_thread, NULL, producer, NULL);
 pthread_create(&consumer_thread, NULL, consumer, NULL);

 
 //wait for the thread to complete

 pthread_join(cpu_thread, NULL);
 pthread_join(cpu_thread2, NULL);
 pthread_join(producer_thread, NULL);
 pthread_join(consumer_thread, NULL);

 free(core1);
 free(core2);
}



int main() {
 loadProgram(); //initialize memory with instructions
 initMemoryTable();
 initCache();

 //initialize mutex for memory access
 pthread_mutex_init(&memory_mutex, NULL);
 sem_init(&cache_semaphore, 0, 1);
 sem_init(&buffer_full, 0, 0); //Buffer starts empty
 sem_init(&buffer_empty, 0, BUFFER_SIZE); //Buffer has BUFFER_SIZE empty slots
 
 startCores();
 //clean up mutex
 pthread_mutex_destroy(&memory_mutex);
 sem_destroy(&cache_semaphore);
 sem_destroy(&buffer_full);
 sem_destroy(&buffer_empty);
 return 0;
}