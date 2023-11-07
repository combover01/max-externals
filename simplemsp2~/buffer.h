#pragma once
//#include <stdbool.h>
#include "ext.h"			// standard Max include, always required (except in Jitter)
#include "ext_obex.h"		// required for "new" style objects
#include "z_dsp.h"			// required for MSP objects


typedef struct ringBuff {
    double** buffer;
    int sampleRate;
    int nChannels;
    int buffLength;
    int writeIndex;
    int readIndex;
    bool isInit;
    bool buffFull;
} ringBuff;

//void InitBuff(ringBuff buff, int nChannel, int buffLength);
//void writeBuff(ringBuff buff, float* samples2write);
//void readBuff(ringBuff buff, float* samples2read);
//void destroyBuff(ringBuff buff);


void InitBuff(ringBuff *buff, const int nChannel, const int buffLength)
{
    buff->nChannels = nChannel;
    buff->buffLength = buffLength * 2;
    post("current buffLength before multiplying: %i", buffLength);
    post("actual buff bufflength: %i", buff->buffLength);
    // https://stackoverflow.com/questions/29977084/calloc-a-two-dimensional-array
    // from vlad from moscow

    buff->buffer = malloc(nChannel * sizeof(double*));
    if (!buff->buffer) {                   /* if you allocate -- you must VALIDATE */
        perror("malloc-m");
        return NULL;
    }
    for (int i = 0; i < nChannel; i++) {
        buff->buffer[i] = calloc(buffLength * 2, sizeof(double));
        if (!buff->buffer[i]) {                   /* if you allocate -- you must VALIDATE */
            perror("calloc-m");
            return NULL;
        }
    }
//    we need readindex always behind writeindex by the delay amount
    buff->readIndex = 0;
    buff->writeIndex = buffLength;
    buff->buffFull = false;
    buff->isInit = true;
}

void writeBuff(ringBuff *buff, float* samples2write)
{
    while (buff->writeIndex >= buff->buffLength)
    {
        buff->writeIndex = buff->writeIndex - buff->buffLength;
        post("writeIndex got wrapped back to %i", buff->writeIndex);
    }
    if (buff->writeIndex < 0){
        buff->writeIndex = buff->writeIndex + buff->buffLength;
        post("writeIndex got wrapped forward to %i", buff->writeIndex);
    }
    for (int i = 0; i < buff->nChannels; i++)
    {
        buff->buffer[i][buff->writeIndex] = samples2write[i];
    }
    post("current writeIndex: %i for buff w readIndex %i and bufflength %i", buff->writeIndex, buff->readIndex, buff->buffLength);
    buff->writeIndex = buff->writeIndex + 1;
    

    if (buff->writeIndex >= buff->buffLength) {
        buff->buffFull = true;
    }
    else {
        buff->buffFull = false;
    }
}

void readBuff(ringBuff *buff, float* samples2read)
{
    for (int i = 0; i < buff->nChannels; i++)
    {
        samples2read[i] = buff->buffer[i][buff->readIndex];
    }
    post("current readIndex: %i", buff->readIndex);
    buff->readIndex++;
    // reading the buffer, one sample (all channels) at a time
    while (buff->readIndex >= buff->buffLength)
    {
        buff->readIndex = buff->readIndex - buff->buffLength;
    }
    if (buff->readIndex < 0){
        buff->readIndex = buff->readIndex + buff->buffLength;
    }

}


void linearInterpolation(ringBuff *inputBuff, ringBuff *outputBuff, float *prevVal) {
    // lastVal is an array of the last values of the last inputBuffer for each channel .... but it is not working as expected!
    //int count = 0;
    int ratio = floor(outputBuff->buffLength / inputBuff->buffLength);
    post("ratio in linearInterpolation: %i", ratio);
    // post("ratio:");
    // post(ratio);

    float* curVal = calloc(inputBuff->nChannels, sizeof(float));
    if (!curVal) {                   /* if you allocate -- you must VALIDATE */
        perror("calloc-m");
        return NULL;
    }
    float* curSamp = calloc(inputBuff->nChannels, sizeof(float));
    if (!curSamp) {                   /* if you allocate -- you must VALIDATE */
        perror("calloc-m");
        return NULL;
    }
    float* diff = calloc(inputBuff->nChannels, sizeof(float));
    if (!diff) {                   /* if you allocate -- you must VALIDATE */
        perror("calloc-m");
        return NULL;
    }
    
    for (int i = 0; i < inputBuff->buffLength; i++) {
        readBuff(inputBuff, curVal);
        post("current inputbuff: %f", curVal[0]);

        // generate audio samples
        for (int k = 0; k < ratio; k++) {
            for (int channel = 0; channel < inputBuff->nChannels; channel++) {
                float step = (curVal[channel]-prevVal[channel])/ratio;
                curSamp[channel] = prevVal[channel] + step;
                prevVal[channel] = curSamp[channel];
            }
//                THIS LINE IS PROB PROBLEM
            post("about to writeToOutputbufff");
            writeBuff(outputBuff, curSamp);
//                post("current outputSample: %f", curSamp[0]);
        }
        
        // set prevVal as last curVal for the loop to start again
        for (int channel = 0; channel < inputBuff->nChannels; channel++) {
            prevVal[channel] = curSamp[channel];
        }
    }
}

void newLinearInterpolation(ringBuff *inputBuff, ringBuff *outputBuff, float *prevVal) {
//    should only be inputting two values and outputting an array of ratio in length
    
    int ratio = floor(outputBuff->buffLength / inputBuff->buffLength);
    // post("ratio:");
    // post(ratio);

    float* curVal = calloc(inputBuff->nChannels, sizeof(float));
    if (!curVal) {                   /* if you allocate -- you must VALIDATE */
        perror("calloc-m");
        return NULL;
    }
    float* curSamp = calloc(inputBuff->nChannels, sizeof(float));
    if (!curSamp) {                   /* if you allocate -- you must VALIDATE */
        perror("calloc-m");
        return NULL;
    }
    float* diff = calloc(inputBuff->nChannels, sizeof(float));
    if (!diff) {                   /* if you allocate -- you must VALIDATE */
        perror("calloc-m");
        return NULL;
    }
    
    
    for (int i = 0; i < inputBuff->buffLength; i++) {
        readBuff(inputBuff, curVal);
       
        // calculate diff array
        for (int channel = 0; channel < inputBuff->nChannels; channel++) {
            // find diff, for each channel
            float f_curVal = curVal[channel];
            float f_prevVal = prevVal[channel];

            // this diff is just between the last and first value
            // divide by the ratio to add it in the audio sample code below
            diff[channel] = (f_curVal - f_prevVal) / (float)ratio;
        }

        // generate audio samples
        for (int k = 0; k < ratio; k++) {
            for (int channel = 0; channel < inputBuff->nChannels; channel++) {
                curSamp[channel] = prevVal[channel] + (diff[channel] * k);
            }
            writeBuff(outputBuff, curSamp);
        }

        // set prevVal as last curVal for the loop to start again
        for (int channel = 0; channel < inputBuff->nChannels; channel++) {
            prevVal[channel] = curVal[channel];
        }
    }
}


void destroyBuff(ringBuff *buff)
{
    for (int i = 0; i < buff->nChannels; i++)
        free(buff->buffer[i]);

    free(buff->buffer);
}

void floatToDoubleBuff(float* floatBuff, double* doubleBuff, int length) {
    // both buffers need to be the same length for this to work
    for (int i = 0; i < length; i++) {
        doubleBuff[i] = (double)floatBuff[i];
    }
}
