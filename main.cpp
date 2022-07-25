#include "mbed.h"
#include "N5110.h"
#include "arm_math.h"   //https://os.mbed.com/teams/mbed-official/code/mbed-dsp/

#define SAMPLES 1024                           //note that the sample taken from the input signal will be reduced by half 
#define NUMBER_OF_OUTPUT_FREQUENCIES SAMPLES/2
#define SAMPLING_FREQUENCY 46875                //sample frequency is 48kHz, however realistically this would around 46.8kHz
#define NUMBER_OF_BANDS 10

//Pin assignment format:  lcd(IO, Ser_TX, Ser_RX, MOSI, SCLK, PWM)  
N5110 lcd(PC_7, PA_9, PB_10, PB_5, PB_3, PA_10);
AnalogIn signal(PA_0);   //scope probe attached to pin PA_0

Ticker sampleTimer;
arm_rfft_fast_instance_f32 fft_handler;

volatile bool g_sampleTimerTrigger;
float FFT_inputBuffer[SAMPLES];
float FFT_outputBuffer[SAMPLES];
uint8_t outarray[14];
int bufferPtr = 0;

void sampleTimer_isr();
void FFT();
float complexABS(float realPart, float imagPart);
void drawWaveform();

int main(){
    lcd.init(LPH7366_1);
    sampleTimer.attach(&sampleTimer_isr, 1ms);
    arm_rfft_fast_init_f32(&fft_handler, SAMPLES);

    while(1){
        if(g_sampleTimerTrigger){
            //clear isr flag
            g_sampleTimerTrigger = false;

            //read pin and store in FFT_inputBuffer
            FFT_inputBuffer[bufferPtr] = signal.read(); //range 0-1

            //once all samples have been taken, pass to FFT()
            if(bufferPtr == SAMPLES){
                FFT();

                //display frequency values on lcd
                drawWaveform();

                //reset buffer pointer
                bufferPtr = 0;

            }else{
                //increment buffer pointer
                bufferPtr++;
            }
        }
    }
}

void sampleTimer_isr(){
    g_sampleTimerTrigger = true;
}
 
void FFT(){
    int frequencies[NUMBER_OF_OUTPUT_FREQUENCIES];
    int frequencyPoint = 0;
    int offset = 150;           //noise offset

    /*Converts the real values of the input signal into the frequency domain.
    * There are 2 modes of operation:
    *                                   Mode 0 -> preforms foreward transformation
    *                                   Mode 1 -> preforms inverse transformation
    * Note that the output array consists of both real (i) and imaginary (i+1) numbers which are considered datasets, 
    * 1024 datasets with 2048 elements in total.
    * arm_rfft_fast_f32(instance, input array, output array, mode)  */
    arm_rfft_fast_f32(&fft_handler, FFT_inputBuffer, FFT_outputBuffer, 0);

    /*Calculate absolute values of the complex output of the arm_rfft_fast_f32 function,
    * before converting them into decibles (dB). This will reduce the number of elements from 2048 to 1024
    * by removing the imaginary part. */
    for(int i = 0; i < 1024; i = i + 2){
        frequencies[frequencyPoint] = (int)(20*log10f(complexABS(FFT_outputBuffer[i], FFT_outputBuffer[i+1])))-offset;
    
        if(frequencies[frequencyPoint] < 0){
            frequencies[frequencyPoint] = 0;
            frequencyPoint++;
        }
    }

    /*To determine which indexes of the outarray we need for specific frequencies,
    * we can use the equation: requiredIndex = requiredFrequency*(NUMBER_OF_OUTPUT_FREQUENCIES/(SAMPLING_FREQUENCY/2)) 
    * Note that there will be a rounding requirement, for example:
    *   requiredIndex = requiredFrequency*(NUMBER_OF_OUTPUT_FREQUENCIES/(SAMPLING_FREQUENCY/2)
    *                 = 500*(1024/(46875/2)) = 21.85 = 22 
    * Due to the rounding of the required index, the frequency that we will be monitoring will not be exactly what we desire.
    * In the example above, we wish to monitor the frequency 500Hz however, due to the rounding of the index we will actually
    * be monitoring 503.5Hz. */

    //require index                      :      required frequency
    outarray[0] = 0xff;                         //frame start
	outarray[1] = (uint8_t)frequencies[1];      //31-5Hz
	outarray[2] = (uint8_t)frequencies[3];      //63 Hz
	outarray[3] = (uint8_t)frequencies[5];      //125 Hz
	outarray[4] = (uint8_t)frequencies[11];     //250 Hz
	outarray[5] = (uint8_t)frequencies[22];     //500 Hz
	outarray[6] = (uint8_t)frequencies[44];     //1 kHz
	outarray[7] = (uint8_t)frequencies[96];     //2.2 kHz
	outarray[8] = (uint8_t)frequencies[197];    //4.5 kHz
	outarray[9] = (uint8_t)frequencies[393];    //9 kHz
	outarray[10] = (uint8_t)frequencies[655];   //15 kHz
}

float complexABS(float realPart, float imagPart){
    return sqrtf(realPart*realPart + imagPart*imagPart);
}

void drawWaveform(){
    int barWidth = 80/NUMBER_OF_BANDS;  //width of each wavelength band display
    int barHeight[NUMBER_OF_BANDS] = {0};
    //int barHeight[NUMBER_OF_BANDS] = {10, 25, 39, 25, 5, 20, 30, 12, 36, 40};

    //calculate the height of each bar indicator
    for(int i = 1; i <= sizeof(outarray); i++){
        /* -Multiplied with the height of Y Axis to scale the reading
        *  -Subtract scaled reading from the end point of the Y Axis to determine Y Coord. */
        barHeight[i] = 41 - (outarray[i]*40);
    }

    lcd.clear();
    lcd.drawLine(2, 41, 81, 41, 1);     //draws baseline (lenght 80px)

    //draw each bar indicator 
    for(int i = 0; i <= NUMBER_OF_BANDS; i++){
        lcd.drawRect(2 + (i * barWidth), 40 - barHeight[i], barWidth, barHeight[i], FILL_BLACK);
    }

    lcd.refresh();
}