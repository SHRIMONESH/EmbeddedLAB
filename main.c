#include "stm32f401xe.h" // Main header for the STM32F401RE microcontroller.
#include <string.h>       // Provides string manipulation functions like strcmp and memset.

// --- Configuration ---
#define LCD_I2C_ADDRESS     0x27        // I2C address for the LCD display.
#define VALID_RFID_UID      "DE4F5A6B\n" // The specific RFID UID that the system will recognize as valid.

// --- MFRC522 Command and Pin Defines ---
// These are commands for the MFRC522 RFID reader module.
#define MFRC522_CMD_IDLE              0x00 // No action; cancels current command execution.
#define MFRC522_CMD_TRANSCEIVE        0x0C // Transmits data to the FIFO buffer, receives data from the card, and vice versa.
#define MFRC522_CMD_SOFT_RESET        0x0F // Resets the MFRC522 module.
#define PICC_CMD_REQIDL               0x26 // Request command for ISO14443A cards.

// Pin control macros for the MFRC522's Chip Select (CS) and Reset (RST) pins.
#define CS_LOW()   (GPIOA->BSRR = (1U << (4 + 16))) // Set PA4 (CS) to low.
#define CS_HIGH()  (GPIOA->BSRR = (1U << 4))        // Set PA4 (CS) to high.
#define RST_LOW()  (GPIOB->BSRR = (1U << (0 + 16))) // Set PB0 (RST) to low.
#define RST_HIGH() (GPIOB->BSRR = (1U << 0))        // Set PB0 (RST) to high.

// --- Global Buffers ---
char received_otp[7] = {0}; // Buffer to store the OTP received from the ESP8266.
char user_otp[7] = {0};     // Buffer to store the OTP entered by the user on the keypad.

// --- Function Prototypes ---
// Prototypes for all functions used in this file, ensuring the compiler knows about them before they are called.
void SystemClock_Config(void);
void SysTick_Init(void);
void DelayMs(uint32_t ms);
void Peripherals_Init(void);
void USART1_SendString(const char* str);
char USART1_ReceiveChar(void);
void LCD_SendCommand(char cmd);
void LCD_SendData(char data);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendString(const char* str);
char Keypad_Scan(void);
void I2C1_Write(uint8_t address, uint8_t *data, uint8_t size);
void rc522_init(void);
uint8_t rc522_check_card(void);
void rc522_write_register(uint8_t addr, uint8_t val);
uint8_t rc522_read_register(uint8_t addr);
uint8_t spi1_transmit_receive(uint8_t data);
uint8_t rc522_to_card(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen);

// --- State Machine Definition ---
// Defines the different states the inventory system can be in.
typedef enum {
    STATE_IDLE,           // Waiting for an RFID card to be scanned.
    STATE_REQUEST_OTP,    // Requesting an OTP from the server.
    STATE_WAIT_FOR_OTP,   // Waiting to receive the OTP.
    STATE_GET_USER_INPUT, // Getting OTP input from the user via the keypad.
    STATE_ACCESS_GRANTED, // Access has been granted.
    STATE_UPDATE_INVENTORY, // Updating the inventory database.
    STATE_ACCESS_DENIED,  // Access has been denied.
    STATE_ERROR,          // An error has occurred.
    STATE_HALT            // A final state to halt the program until reset.
} SystemState;

volatile uint32_t msTicks = 0; // A global variable to count milliseconds for the delay function.

// SysTick interrupt handler, called every millisecond.
void SysTick_Handler(void) { msTicks++; }

// --- MAIN PROGRAM ---
int main(void) {
    // Initial setup for the microcontroller and peripherals.
    SystemClock_Config();
    SysTick_Init();
    Peripherals_Init();
    LCD_Init();
    rc522_init();

    // The state machine starts in the IDLE state.
    SystemState currentState = STATE_IDLE;

    while (1) { // The main infinite loop.
        switch (currentState) {
            case STATE_IDLE:
                // Display a welcome message and wait for a card scan.
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("Smart Inventory");
                LCD_SetCursor(1, 0);
                LCD_SendString("Scan Your Card");

                DelayMs(10000); // Wait for 10 seconds.
                currentState = STATE_REQUEST_OTP; // Move to the next state.
                break;

            case STATE_REQUEST_OTP:
                // Inform the user that the system is retrieving an OTP.
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("System Triggered");
                LCD_SetCursor(1, 0);
                LCD_SendString("Retrieving OTP...");
                DelayMs(3000);

                // Send the valid RFID UID to the ESP8266 to request an OTP.
                USART1_SendString(VALID_RFID_UID);
                currentState = STATE_WAIT_FOR_OTP; // Move to the next state.
                break;

            case STATE_WAIT_FOR_OTP:
                LCD_Clear();
                LCD_SetCursor(0,0);
                LCD_SendString("Waiting for OTP...");

                // Receive the 6-digit OTP from the ESP8266.
                for (int i = 0; i < 6; i++) {
                    received_otp[i] = USART1_ReceiveChar();
                }
                received_otp[6] = '\0'; // Null-terminate the string.

                // If "000000" is received, it's an error.
                if (strcmp(received_otp, "000000") == 0) {
                    currentState = STATE_ERROR;
                } else {
                    LCD_Clear();
                    LCD_SetCursor(0, 0);
                    LCD_SendString("OTP Retrieved!");
                    DelayMs(3000);
                    currentState = STATE_GET_USER_INPUT; // OTP received, now get user input.
                }
                break;

            case STATE_GET_USER_INPUT:
                // Prompt the user to enter the OTP.
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("Enter OTP:");

                memset(user_otp, 0, sizeof(user_otp)); // Clear the user OTP buffer.
                int otp_count = 0;
                LCD_SetCursor(1, 0);
                LCD_SendString("                "); // Clear the second line of the LCD.
                LCD_SetCursor(1, 0);

                // Collect 6 digits from the keypad.
                while (otp_count < 6) {
                    char key = Keypad_Scan();
                    if (key != 0) {
                        user_otp[otp_count] = key;
                        LCD_SendData(key); // Display the entered digit.
                        otp_count++;
                        DelayMs(200); // Debounce delay.
                    }
                }
                user_otp[6] = '\0'; // Null-terminate the string.

                // Check if the entered OTP matches the received OTP.
                if (strcmp(user_otp, received_otp) == 0) {
                    currentState = STATE_ACCESS_GRANTED;
                } else {
                    currentState = STATE_ACCESS_DENIED;
                }
                break;

            case STATE_ACCESS_GRANTED:
                // Inform the user that access is granted.
                LCD_Clear(); LCD_SetCursor(0, 0); LCD_SendString("Access Granted");
                DelayMs(5000);
                LCD_Clear(); LCD_SetCursor(0, 0); LCD_SendString("Access Granted");
                LCD_SetCursor(1, 0); LCD_SendString("Press Btn-> Add");
                // Wait for the user to press the blue button (PC13).
                while (GPIOC->IDR & GPIO_IDR_ID13);
                DelayMs(50);
                while (!(GPIOC->IDR & GPIO_IDR_ID13));
                currentState = STATE_UPDATE_INVENTORY; // Move to the inventory update state.
                break;

            case STATE_UPDATE_INVENTORY:
                // Simulate updating the database.
                LCD_Clear(); LCD_SetCursor(0, 0); LCD_SendString("Updating DB...");
                DelayMs(5000);
                USART1_SendString("UPDATE_DB\n"); // Send a command to the ESP8266 to update the database.
                LCD_Clear(); LCD_SetCursor(0, 0); LCD_SendString("1 watch added!");
                DelayMs(5000);
                currentState = STATE_HALT; // End the sequence.
                break;

            case STATE_ACCESS_DENIED:
                // Inform the user that access is denied.
                LCD_Clear(); LCD_SetCursor(0, 0); LCD_SendString("Incorrect OTP!");
                LCD_SetCursor(1, 0); LCD_SendString("Access Denied");
                DelayMs(5000);
                currentState = STATE_HALT; // End the sequence.
                break;

            case STATE_ERROR:
                // Inform the user that an error has occurred.
                LCD_Clear(); LCD_SetCursor(0, 0); LCD_SendString("Error: No OTP");
                LCD_SetCursor(1, 0); LCD_SendString("Check ESP/Server");
                DelayMs(5000);
                currentState = STATE_HALT; // End the sequence.
                break;

            case STATE_HALT:
                // The final state. The program will do nothing until it is reset.
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_SendString("Sequence End.");
                LCD_SetCursor(1, 0);
                LCD_SendString("Please Reset.");
                while(1) {
                    // Infinite loop to halt the processor.
                }
                break;
        }
    }
}

/******************************************************************************
 * All peripheral and driver functions are below this line.
 ******************************************************************************/

// Initializes all the necessary peripherals for the project.
void Peripherals_Init(void) {
    // Enable clocks for GPIOA, GPIOB, GPIOC, I2C1, USART1, and SPI1.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SPI1EN;
    // Configure PA9 (USART1_TX) and PA10 (USART1_RX) as alternate function.
    GPIOA->MODER &= ~((3U << (9*2)) | (3U << (10*2)));
    GPIOA->MODER |= (2U << (9*2)) | (2U << (10*2));
    GPIOA->AFR[1] |= (7U << GPIO_AFRH_AFSEL9_Pos) | (7U << GPIO_AFRH_AFSEL10_Pos);
    // Configure PB8 (I2C1_SCL) and PB9 (I2C1_SDA) as alternate function, open-drain, with pull-up resistors.
    GPIOB->MODER &= ~((3U << (8*2)) | (3U << (9*2)));
    GPIOB->MODER |= (2U << (8*2)) | (2U << (9*2));
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    GPIOB->PUPDR |= ((1U << (8*2)) | (1U << (9*2)));
    GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL8_Pos) | (4U << GPIO_AFRH_AFSEL9_Pos);
    // Configure keypad rows (PB4-PB7) as output.
    GPIOB->MODER &= ~((3U << (4*2)) | (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOB->MODER |= ((1U << (4*2)) | (1U << (5*2)) | (1U << (6*2)) | (1U << (7*2)));
    // Configure keypad columns (PC0-PC3) as input with pull-up resistors.
    GPIOC->MODER &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2)) | (3U << (3*2)));
    GPIOC->PUPDR &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2)) | (3U << (3*2)));
    GPIOC->PUPDR |= ((1U << (0*2)) | (1U << (1*2)) | (1U << (2*2)) | (1U << (3*2)));
    // Configure SPI pins: PA4 (CS) as output, PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function.
    GPIOA->MODER &= ~((3U << (4*2)) | (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->MODER |= (1U << (4*2));
    GPIOA->MODER |= (2U << (5*2)) | (2U << (6*2)) | (2U << (7*2));
    GPIOA->AFR[0] |= (5U << GPIO_AFRL_AFSEL5_Pos) | (5U << GPIO_AFRL_AFSEL6_Pos) | (5U << GPIO_AFRL_AFSEL7_Pos);
    // Configure RFID reset pin (PB0) as output.
    GPIOB->MODER &= ~(3U << (0*2));
    GPIOB->MODER |= (1U << (0*2));
    CS_HIGH(); // Set Chip Select high to de-select the RFID module.
    // Configure the blue user button (PC13) as input.
    GPIOC->MODER &= ~GPIO_MODER_MODE13;
    // Configure USART1 with a baud rate of 9600 (for a 16MHz clock).
    USART1->BRR = 0x683;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable transmitter, receiver, and USART.
    // Configure I2C1 for 100kHz communication speed.
    I2C1->CR1 |= I2C_CR1_SWRST; I2C1->CR1 &= ~I2C_CR1_SWRST; // Reset and then enable I2C.
    I2C1->CR2 |= (16 << 0);
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
    // Configure SPI1 as master, with a prescaler of 8.
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (3 << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI.
}
// Configures the system clock to use the internal 16MHz HSI oscillator.
void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION; // Enable HSI.
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait for HSI to be ready.
    RCC->CFGR &= ~RCC_CFGR_SW; // Select HSI as the system clock source.
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait for the clock switch to complete.
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_0WS; // Configure flash latency.
}
// Initializes the SysTick timer to generate an interrupt every millisecond.
void SysTick_Init(void) { SysTick_Config(16000000 / 1000); }
// A simple blocking delay function that uses the SysTick timer.
void DelayMs(uint32_t ms) { uint32_t startTicks = msTicks; while ((msTicks - startTicks) < ms); }
// Sends a null-terminated string over USART1.
void USART1_SendString(const char* str) { while (*str) { while (!(USART1->SR & USART_SR_TXE)); USART1->DR = *str++; } }
// Receives a single character from USART1.
char USART1_ReceiveChar(void) { while (!(USART1->SR & USART_SR_RXNE)); return (char)USART1->DR; }
// Sends a block of data over I2C1.
void I2C1_Write(uint8_t address, uint8_t *data, uint8_t size) {
    while (I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START; while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = address << 1; while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    for (uint8_t i = 0; i < size; i++) { I2C1->DR = data[i]; while (!(I2C1->SR1 & I2C_SR1_TXE)); }
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}
// Sends a command to the LCD.
void LCD_SendCommand(char cmd) {
    char d[4]; d[0]=(cmd&0xF0)|0x0C; d[1]=(cmd&0xF0)|0x08; d[2]=((cmd<<4)&0xF0)|0x0C; d[3]=((cmd<<4)&0xF0)|0x08;
    I2C1_Write(LCD_I2C_ADDRESS, (uint8_t*)d, 4);
}
// Sends data (a character) to the LCD.
void LCD_SendData(char data) {
    char d[4]; d[0]=(data&0xF0)|0x0D; d[1]=(data&0xF0)|0x09; d[2]=((data<<4)&0xF0)|0x0D; d[3]=((data<<4)&0xF0)|0x09;
    I2C1_Write(LCD_I2C_ADDRESS, (uint8_t*)d, 4);
}
// Initializes the LCD.
void LCD_Init(void) {
    DelayMs(50); LCD_SendCommand(0x30); DelayMs(5); LCD_SendCommand(0x30); DelayMs(1);
    LCD_SendCommand(0x30); DelayMs(10); LCD_SendCommand(0x20); DelayMs(10);
    LCD_SendCommand(0x28); DelayMs(1); LCD_SendCommand(0x08); DelayMs(1);
    LCD_SendCommand(0x01); DelayMs(2); LCD_SendCommand(0x06); DelayMs(1);
    LCD_SendCommand(0x0C);
}
// Clears the LCD display.
void LCD_Clear(void) { LCD_SendCommand(0x01); DelayMs(5); }
// Sets the cursor position on the LCD.
void LCD_SetCursor(uint8_t row, uint8_t col) { LCD_SendCommand((row==0)?(0x80+col):(0xC0+col)); }
// Sends a string to be displayed on the LCD.
void LCD_SendString(const char* str) { while (*str) { LCD_SendData(*str++); } }
// Scans the keypad and returns the character of the pressed key.
char Keypad_Scan(void) {
    const char keymap[4][4] = {{'1','2','3','A'},{'4','5','6','B'},{'7','8','9','C'},{'*','0','#','D'}};
    for (int r=0; r<4; r++) {
        GPIOB->BSRR = (1U<<4)|(1U<<5)|(1U<<6)|(1U<<7); GPIOB->BSRR = (1U<<((r+4)+16));
        for (int c=0; c<4; c++) {
            if(!(GPIOC->IDR&(1U<<c))) {
                DelayMs(50); while(!(GPIOC->IDR&(1U<<c))); return keymap[r][c];
            }
        }
    }
    return '\0'; // Return null character if no key is pressed.
}
// Writes a value to a register of the MFRC522.
void rc522_write_register(uint8_t addr, uint8_t val) {
    CS_LOW(); spi1_transmit_receive((addr<<1)&0x7E); spi1_transmit_receive(val); CS_HIGH();
}
// Reads a value from a register of the MFRC522.
uint8_t rc522_read_register(uint8_t addr) {
    uint8_t val; CS_LOW(); spi1_transmit_receive(((addr<<1)&0x7E)|0x80);
    val = spi1_transmit_receive(0x00); CS_HIGH(); return val;
}
// Initializes the MFRC522 RFID reader.
void rc522_init(void) {
    RST_HIGH(); RST_LOW(); DelayMs(1); RST_HIGH(); DelayMs(10);
    rc522_write_register(0x01, MFRC522_CMD_SOFT_RESET); DelayMs(10);
    while (rc522_read_register(0x01) & 0x08);
    rc522_write_register(0x2A, 0x8D); rc522_write_register(0x2B, 0x3E);
    rc522_write_register(0x2D, 30); rc522_write_register(0x2C, 0);
    rc522_write_register(0x15, 0x40); rc522_write_register(0x11, 0x03);
    rc522_write_register(0x14, 0x88); // Antenna On
}
// Communicates with an RFID card.
uint8_t rc522_to_card(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen) {
    uint8_t status = 2; uint8_t irqEn = 0x77; uint8_t waitIRq = 0x30;
    uint8_t n; uint16_t i;
    rc522_write_register(0x02, irqEn|0x80); rc522_write_register(0x04, 0x7F);
    rc522_write_register(0x0A, 0x80); rc522_write_register(0x01, MFRC522_CMD_IDLE);
    for (i=0;i<sendLen;i++) { rc522_write_register(0x09, sendData[i]); }
    rc522_write_register(0x01, command);
    if (command == MFRC522_CMD_TRANSCEIVE) { rc522_write_register(0x0D, 0x87); }
    i=2000; while(1) { n=rc522_read_register(0x04); i--; if(~(i!=0 && ~(n&0x01) && ~(n&waitIRq))) { break; } }
    rc522_write_register(0x0D, 0x00);
    if(i!=0) {
        if(!(rc522_read_register(0x06)&0x1B)) {
            status = 0;
            if(n&irqEn&0x01){ status = 1; }
            if(command==MFRC522_CMD_TRANSCEIVE) {
                n=rc522_read_register(0x0A); uint8_t lastBits=rc522_read_register(0x0C)&0x07;
                if(lastBits!=0) { *backLen=(n-1)*8+lastBits; } else { *backLen=n*8; }
                if(n==0){n=1;} if(n>16){n=16;}
                for(i=0;i<n;i++){ backData[i]=rc522_read_register(0x09); }
            }
        } else { status = 2; }
    }
    return status;
}
// Checks for the presence of an RFID card.
uint8_t rc522_check_card(void) {
    uint8_t status; uint8_t str[16]; uint16_t recvBits;
    uint8_t buffer[1] = {PICC_CMD_REQIDL};
    rc522_write_register(0x08, 0x99);
    status = rc522_to_card(MFRC522_CMD_TRANSCEIVE, buffer, 1, str, &recvBits);
    return (status == 0 && recvBits == 0x10);
}
// Transmits and receives a byte of data over SPI1.
uint8_t spi1_transmit_receive(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}
