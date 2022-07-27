//hardware fault relates to drawWaveform();

#include "mbed.h"
#include "N5110.h"
#include "arm_math.h"   //https://os.mbed.com/teams/mbed-official/code/mbed-dsp/

#define SAMPLES 2048                           //note that the sample taken from the input signal will be reduced by half 
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
uint8_t outarray[NUMBER_OF_BANDS + 1];
int bufferPtr = 0;

void sampleTimer_isr();
void FFT();
float complexABS(float realPart, float imagPart);
void drawWaveform();

int main(){
    lcd.init(LPH7366_1);
    sampleTimer.attach(&sampleTimer_isr, 2ms);    //500us
    arm_rfft_fast_init_f32(&fft_handler, SAMPLES);

    while(1){
        if(g_sampleTimerTrigger){
            //clear isr flag
            g_sampleTimerTrigger = false;

            //read pin and store in FFT_inputBuffer
            FFT_inputBuffer[bufferPtr] = signal.read(); //range 0-1

            //debug function
            printf("\nDebug >> FFT inputBuffer %d: %.2f", bufferPtr, FFT_inputBuffer[bufferPtr]);

            //once all samples have been taken, pass to FFT()
            if(bufferPtr == 2047){
                printf("\n\nDebug >> bufferPtr = 2047; begin FFT");//debug function
                ThisThread::sleep_for(1ms);
                
                FFT();
                printf("\n\nDebug >> FFT Complete; draw waveform"); //debug function
                ThisThread::sleep_for(1ms);

                //display frequency values on lcd
                drawWaveform();
                //reset buffer pointer
                bufferPtr = 0;
                printf("\nDebug >> waveform complete; bufferPtr = %d", bufferPtr); //debug function
                ThisThread::sleep_for(1ms);

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
    
    //debug function
    for(int i = 0; i == sizeof(FFT_outputBuffer); i = i+2){
        printf("\nDebug >> FFT_outputBuffer %d: %.2f", i, FFT_outputBuffer[i]);
    }

    ThisThread::sleep_for(1ms);

    /*Calculate absolute values of the complex output of the arm_rfft_fast_f32 function,
    * before converting them into decibles (dB). This will reduce the number of elements from 2048 to 1024
    * by removing the imaginary part. */
    for(int i = 0; i <= 2045; i = i + 2){
        frequencies[frequencyPoint] = (int)(20*log10f(complexABS(FFT_outputBuffer[i], FFT_outputBuffer[i+1])))-offset;
        printf("\nDebug >> Attained frequencies %d: %.2d", frequencyPoint, frequencies[frequencyPoint]); //debug function
    
        if(frequencies[frequencyPoint] < 0){
            frequencies[frequencyPoint] = 0;
        }
        
        frequencyPoint++;
        ThisThread::sleep_for(1ms);
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

    //debug function
    for(int i = 1; i <= 10; i++){
        printf("\nDebug >> Outarray %d: %.2d", i, outarray[i]);
    }
    ThisThread::sleep_for(1ms);

}

float complexABS(float realPart, float imagPart){
    return sqrtf(realPart*realPart + imagPart*imagPart);
}

void drawWaveform(){
    int barWidth = 80/NUMBER_OF_BANDS;  //width of each wavelength band display
    int barHeight[NUMBER_OF_BANDS + 1] = {0};
    printf("\nDebug >> barWidth: %.2d", barWidth);
    ThisThread::sleep_for(1ms);

    //calculate the height of each bar indicator
    for(int i = 0; i <= sizeof(outarray); i++){
        /* -Multiplied with the height of Y Axis to scale the reading
        *  -Subtract scaled reading from the end point of the Y Axis to determine Y Coord. */
        barHeight[i] = outarray[i];

        //enforce limits
        if(barHeight[i] < 0){
            barHeight[i] = 0;

        }else if(barHeight[i] > 40){
            barHeight[i] = 40;
        }
        printf("\nDebug >> barHeight %d: %.2d", i, barHeight[i]);
        ThisThread::sleep_for(1ms);
    }

    lcd.clear();
    lcd.drawLine(2, 41, 83, 41, 1); //draws baseline (lenght 80px)

    //draw bars, ignoring barheight[0] (outarray[0])
    lcd.drawRect(2, 40 - barHeight[1], barWidth, barHeight[1], FILL_BLACK);                     //draw bar 1
    lcd.drawRect(2 + (1 * barWidth), 40 - barHeight[2], barWidth, barHeight[2], FILL_BLACK);    //draw bar 2
    lcd.drawRect(2 + (2 * barWidth), 40 - barHeight[3], barWidth, barHeight[3], FILL_BLACK);    //draw bar 3
    lcd.drawRect(2 + (3 * barWidth), 40 - barHeight[4], barWidth, barHeight[4], FILL_BLACK);    //draw bar 4
    lcd.drawRect(2 + (4 * barWidth), 40 - barHeight[5], barWidth, barHeight[5], FILL_BLACK);    //draw bar 5
    lcd.drawRect(2 + (5 * barWidth), 40 - barHeight[6], barWidth, barHeight[6], FILL_BLACK);    //draw bar 6
    lcd.drawRect(2 + (6 * barWidth), 40 - barHeight[7], barWidth, barHeight[7], FILL_BLACK);    //draw bar 7
    lcd.drawRect(2 + (7 * barWidth), 40 - barHeight[8], barWidth, barHeight[8], FILL_BLACK);    //draw bar 8
    lcd.drawRect(2 + (8 * barWidth), 40 - barHeight[9], barWidth, barHeight[9], FILL_BLACK);    //draw bar 9
    lcd.drawRect(2 + (9 * barWidth), 40 - barHeight[10], barWidth, barHeight[10], FILL_BLACK);  //draw bar 10

    lcd.refresh();
    ThisThread::sleep_for(1s);
}