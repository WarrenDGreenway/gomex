/**************************************************************************
    PygmyOS ( Pygmy Operating System )
    Copyright (C) 2011-2012  Warren D Greenway

    This file is part of PygmyOS.

    PygmyOS is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PygmyOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with PygmyOS.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
#include "pygmy_profile.h"
#include "pygmy_rtc.h"
#include "core/pygmy_pin.h"
#include "core/pygmy_fifo.h"
#include "core/pygmy_stream.h"
#include "pygmy_uart.h"
#include "core/stm32.h"
#include "pygmy_string.h"
#include "pygmy_clock.h"
#include "pygmy_console.h"

using namespace std;

#define BOOT_CANCEL         BIT7
#define BOOT_NOOS           BIT6
#define BOOT_BAUDRATE       115200

#define LOOPBACK            0x00
#define RS232RS485          0x01 // LOW LOW HIGH
#define RS232               0x01 // LOW LOW HIGH
#define RS422_FULLDUPLEX    0x05 // HIGH LOW HIGH
#define RS422_HALFDUPLEX    0x04 // HIGH LOW LOW
#define RS485_FULLDUPLEX    0x03 // LOW HIGH HIGH
#define RS485_HALFDUPLEX    0x02 // LOW HIGH LOW 



const u8 STRID_L15X[] = "L15x";
const u8 STRID_F100[] = "F100";
const u8 STRID_F103[] = "F103";

volatile u32 globalBootTimeout;
volatile u32 globalPLL, globalID, globalXTAL, globalFreq, ulClock;
volatile u8 *globalStrID;
volatile u8 globalStatus = 0, globalBootStatus = 0;
//Uart com1;

void setUARTMode( u8 Mode )
{
    /*if ( Mode == LOOPBACK ) {
        pinSet( MODE0, LOW );
        pinSet( MODE1, LOW );
        pinSet( MODE2, LOW );
    } else if( Mode == RS232RS485 ){
        pinSet( MODE0, LOW );
        pinSet( MODE1, LOW );
        pinSet( MODE2, HIGH );
    } else if( Mode == RS232 ){
        pinSet( MODE0, LOW );
        pinSet( MODE1, LOW );
        pinSet( MODE2, HIGH );
    } else if( Mode == RS422_FULLDUPLEX ){
        pinSet( MODE0, HIGH );
        pinSet( MODE1, LOW );
        pinSet( MODE2, HIGH );
    } else if( Mode == RS422_HALFDUPLEX ){
        pinSet( MODE0, HIGH );
        pinSet( MODE1, LOW );
        pinSet( MODE2, LOW );
    } else if( Mode == RS485_FULLDUPLEX ){
        pinSet( MODE0, LOW );
        pinSet( MODE1, HIGH );
        pinSet( MODE2, HIGH );
    } else if( Mode == RS485_HALFDUPLEX ){
        pinSet( MODE0, LOW );
        pinSet( MODE1, HIGH );
        pinSet( MODE2, LOW );
    } // else if
    */
}



int main( void )
{
    u32 testMode, testClear;
    Clock clock( 72000000 ); // setclock

    Pin pinLED1( LED1, OUT );
    Pin pinLED2( LED2, OUT );
    Pin pinBAT1( BAT1, PULLUP );
    Pin pinBAT2( BAT2, PULLUP );
    Pin pinBT_CONNECT( BT_CONNECT, PULLUP );
    Pin pinMCOM1MODE0( MCOM1_MODE0, OUT );
    Pin pinMCOM1MODE1( MCOM1_MODE1, OUT );
    Pin pinMCOM1MODE2( MCOM1_MODE2, OUT );

    Pin pinMCOM2MODE0( MCOM2_MODE0, OUT );
    Pin pinMCOM2MODE1( MCOM2_MODE1, OUT );
    Pin pinMCOM2MODE2( MCOM2_MODE2, OUT );
    Pin pinCURRENT_OUT( CURRENT_OUT, ANALOG );
    Pin pinVOUT1( VOUT1, ANALOG );
    Pin pinVOUT2( VOUT2, ANALOG );
    Pin pinVOUT3( VOUT3, ANALOG );
    Pin pinVOUT4( VOUT4, ANALOG );
    Pin pinVOUT5( VOUT5, ANALOG );
    //Pin pinBT_RESET( BT_RESET, OUT );
    Pin pinResEnable( ENABLE_RES, OUT );
    
    // Initialize Watchdog
    /*PYGMY_WATCHDOG_UNLOCK;
    PYGMY_WATCHDOG_PRESCALER( IWDT_PREDIV128 );
    PYGMY_WATCHDOG_TIMER( 0x0FFF );
    PYGMY_WATCHDOG_START;
    PYGMY_WATCHDOG_REFRESH;*/
    // End Initialize Watchdog
    
    
    
    //pinInterrupt( (void*)handleBTConnectButton, BT_CONNECT, TRIGGER_RISING|TRIGGER_FALLING, 2 );

    //adcEnableChannel( VOLTAGE_OUT );
    //adcEnableChannel( CURRENT_OUT );
    //adcSingleSampleInit();
    // End Initialize Pins
    // Initialize COM ports 

    /*setUARTMode( RS485_HALFDUPLEX );

    Pin pinCOM1_RX( COM1_RX, IN );
    Pin pinCOM1_TX( COM1_TX, ALT );
    Pin pinCOM1_RTS( COM1_RTS, IN );
    Pin pinCOM1_CTS( COM1_CTS, ALT );
    //pinSet( (u16)COM1_CTS, (u8)LOW );
    USART1->BRR = ( ( (ulClock >> 3 ) / 115200 ) << 4 ) + ( ( ( ulClock / 115200 ) ) & 0x0007 );
    USART1->CR3 = USART_ONEBITE;
    USART1->CR1 = ( USART_OVER8 | USART_UE | USART_RXNEIE | USART_TE | USART_RE | USART_RTSE | USART_CTSE );

    Pin pinCOM2_RX( COM2_RX, IN );
    Pin pinCOM2_TX( COM2_TX, ALT );
    Pin pinCOM2_RTS( COM2_RTS, IN );
    Pin pinCOM2_CTS( COM2_CTS, OUT );
    USART2->BRR = ( ( (ulClock >> 3 ) / 115200 ) << 4 ) + ( ( ( ulClock / 115200 ) ) & 0x0007 );
    USART2->CR3 = USART_ONEBITE;
    USART2->CR1 = ( USART_OVER8 | USART_UE | USART_RXNEIE | USART_TE | USART_RE | USART_RTSE | USART_CTSE );

    Pin pinCOM3_RX( COM3_RX, IN );
    Pin pinCOM3_TX( COM3_TX, ALT );
    Pin pinCOM3_RTS( COM3_RTS, ALT );
    Pin pinCOM3_CTS( COM3_CTS, ALT );
    Pin pinCOM3_RI( COM3_RI, PULLUP );
    Pin pinCOM3_DTR( COM3_DTR, OUT );
    Pin pinCOM3_DCD( COM3_DCD, OUT );
    USART3->BRR = ( ( (ulClock >> 3 ) / 115200 ) << 4 ) + ( ( ( ulClock / 115200 ) ) & 0x0007 );
    USART3->CR3 = USART_ONEBITE;
    USART3->CR1 = ( USART_OVER8 | USART_UE | USART_RXNEIE | USART_TE | USART_RE ); 
    
    Pin pinCOM4_RX( COM4_RX, IN );
    Pin pinCOM4_TX( COM4_TX, ALT );
    USART4->BRR = ( ( (ulClock >> 3 ) / 115200 ) << 4 ) + ( ( ( ulClock / 115200 ) ) & 0x0007 );
    USART4->CR3 = USART_ONEBITE;
    USART4->CR1 = ( USART_OVER8 | USART_UE | USART_RXNEIE | USART_TE | USART_RE );
    // End Initialize COM Ports
    */
    // Initialize Streams
    /*streamInit();
    streamSetPut( COM1, (void*)putsUSART1 );
    streamSetPutc( COM1, (void*)putcUSART1 );

    streamSetPut( COM2, (void*)putsUSART2 );
    streamSetPutc( COM1, (void*)putcUSART2 );

    streamSetPut( COM3, (void*)putsUSART3 );
    streamSetPutc( COM3, (void*)putcUSART3 );

    streamSetPut( COM4, (void*)putsUSART4 );
    streamSetPutc( COM4, (void*)putcUSART4 );

    streamSetSTDIO( COM1 );*/
    // End Initialize Streams
    
    // Print Boot Greeting and Specs
    // Initialize the RTC
    //timeInit();
    // End Initialize the RTC
    // Initialize the XModem Interface
    //globalBootStatus = BOOT_CANCEL;
    
    NVIC->ISER[ 1 ] = 0x00000001 << 7;
    NVIC->ISER[ 1 ] = 0x00000001 << 6;
    NVIC->ISER[ 1 ] = 0x00000001 << 5;
    SYSTICK->VAL = globalFreq / 1000;
    SYSTICK->LOAD = globalFreq / 1000; // Based on  ( 2X the System Clock ) / 1000
    SYSTICK->CTRL = 0x07;   // Enable system timer
    // End Initialize Interrupts
    
    //testMode = convertStringToInt( (unsigned char*)"Hello World" );
    pinLED1.set( true );
    pinLED2.set( true );

    pinMCOM1MODE0.set( false );
    pinMCOM1MODE1.set( false );
    pinMCOM1MODE2.set( true );

    pinMCOM2MODE0.set( false );
    pinMCOM2MODE1.set( false );
    pinMCOM2MODE2.set( true );

    pinResEnable.set( false ); // Disable resistance measurement 

    adcEnableChannel( VOUT1 );
    adcEnableChannel( VOUT2 );
    adcEnableChannel( VOUT3 );
    adcEnableChannel( VOUT4 );
    adcEnableChannel( VOUT5 );
    adcEnableChannel( ADCTEMP);
    adcEnableChannel( CURRENT_OUT );
    adcSingleSampleInit();

    Console console( Console::COM1, 1, COM1_TX, COM1_RX, COM1_CTS, COM1_RTS );
    Uart com2( Uart::COM2, 1, COM2_TX, COM2_RX, COM2_CTS, COM2_RTS );
    Uart com3( Uart::COM3, 1, COM3_TX, COM3_RX, COM3_CTS, COM3_RTS );
    Uart com4( Uart::COM4, 1, COM4_TX, COM4_RX, 0, 0 );

    //com3.setPipe( COM1 );
    //com1.setPipe( COM3 );
    //com1.print( "\nCOM1: %d %d %d %d %d", pinVOUT1.analog(), pinVOUT2.analog(), pinVOUT3.analog(), pinVOUT4.analog(), pinVOUT5.analog() );
    
    //PygmyString s( "\nCOM1: " + pinVOUT1.analog() + pinVOUT2.analog() + pinVOUT3.analog() );
    
    //com1.print( "\nVolt1: %d", v1 ); //adcSingleSample( VOUT1 ), adcSingleSample( VOUT2 ), 
        //pinVOUT3.analog(), pinVOUT4.analog(), pinVOUT5.analog() );
    //console.print( "\ntesting" ); //"\nCURRENT: %d", adcSingleSample( CURRENT_OUT ) );
    //console.print( "\nsize" );
    console.init();
    //console.print( "\\" );
    com2.print( "\nCOM2: Hello World!" );
    com3.print( "\nCOM3: Hello World!" );
    com4.print( "\nCOM4: Hello World!" );
    
    while( 1 ){
        // Wait for commands
        /*int v1 = 0;
        for( int i = 0; i < 60; i++ ){
            v1 += adcSingleSample( VOUT1 );
        } // for
        v1 /= 60;
        com1.print( "\nVolt: %d", v1 );*/
        //, adcSingleSample( VOUT2 ), pinVOUT3.analog(), pinVOUT4.analog(), pinVOUT5.analog() );
    }
}

/*#ifdef __cplusplus
extern "C" {
#endif

void USART1_IRQHandler( void )
{
    u16 i;
    char c;

    c= USART1->DR;
    for( i = 0; !( USART1->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART1->DR = c;
}
#ifdef __cplusplus
}
#endif
*/
/*
void USART1_IRQHandler( void )
{
    static u8 ucBuffer[ 256 ], ucIndex = 0;
    u8 ucByte, CurrentStream;

    //CurrentStream = streamGetSTDIO();
    //streamSetSTDIO( COM1 );
    if( USART1->SR & USART_RXNE ) { 
        USART1->SR &= ~USART_RXNE;
        ucByte = USART1->DR ;
        USART1->DR = ucByte;
        //if( xmodemProcess( &XModem, ucByte ) ){
        //    streamSetSTDIO( CurrentStream );
        //    return;
        //} // if
        //if( ucByte == '+' ){
        //    taskDelete( "boot" );
        //    globalBootStatus = BOOT_CANCEL;
        //    //putstr( (u8*)BOOT_PROMPT );
        //    //print( COM3, BOOT_PROMPT );
        //    bootPrintPrompt();
        //    streamSetSTDIO( CurrentStream );
        //    return;
        //} // if
        //if( globalBootStatus & BOOT_CANCEL ){
            if( ucByte == '\r' ){
                ucBuffer[ ucIndex ] = '\0'; // Add NULL to terminate string
                ucIndex = 0;
                if( executeCmd( ucBuffer, (PYGMYCMD *)BOOTCOMMANDS ) ){
                    //print( COM3, BOOT_PROMPT  ); 
                    bootPrintPrompt();
                } else{
                    //print( COM3, BOOT_ERROR ); 
                    com1.print( STDIO, "\rERROR" );
                    bootPrintPrompt();
                } // else
            } else{
                //putcUSART3( ucByte );
                //print( COM3, "%c", ucByte );
                USART1->DR = ucByte;
                if( ucByte == '\b'  ){
                    if( ucIndex ){
                        --ucIndex;
                    } // if
                } else {
                    ucBuffer[ ucIndex++ ] = ucByte;
                } // else
            } // else
        //} // if
    } // if
    streamSetSTDIO( CurrentStream );
}*/

extern "C" void abort(void)
{
  while (1);
}
