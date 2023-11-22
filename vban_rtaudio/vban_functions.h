#ifndef VBAN_FUNCTIONS_H
#define VBAN_FUNCTIONS_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vban.h"

#ifdef __linux__
#include <alsa/asoundlib.h>
#endif


inline uint16_t int16betole(u_int16_t input)
{
    return ((((uint8_t*)&input)[0])<<8) + ((uint8_t*)&input)[1];
}


inline uint32_t int32betole(uint32_t input)
{
    return (((((((uint8_t*)&input)[0]<<8)+((uint8_t*)&input)[1])<<8)+((uint8_t*)&input)[2])<<8)+((uint8_t*)&input)[3];
}


inline void inc_nuFrame(VBanHeader* header)
{
    header->nuFrame++;
}


inline void convertSampleTX(uint8_t* ptr, float sample, uint8_t bit_fmt)
{
    int32_t tmp = 0;
    uint32_t* tp;

    switch (bit_fmt)
    {
    case 0: //VBAN_BITFMT_8_INT:
        tmp = (int8_t)roundf((float)(1<<7)*sample); //((float)(1<<7)*sample);//
        ptr[0] = tmp&0xFF;
        break;
    case 1: //VBAN_BITFMT_16_INT:
        tmp = (int16_t)roundf((float)(1<<15)*sample); //((float)(1<<15)*sample);//
        ptr[0] = tmp&0xFF;
        ptr[1] = (tmp>>8)&0xFF;
        break;
    case 2: //VBAN_BITFMT_24_INT:
        tmp = (int32_t)roundf((float)(1<<23)*sample); //((float)(1<<23)*sample);//
        ptr[0] = tmp&0xFF;
        ptr[1] = (tmp>>8)&0xFF;
        ptr[2] = (tmp>>16)&0xFF;
        break;
    case 3: //VBAN_BITFMT_32_INT:
        ptr[0] = tmp&0xFF;
        ptr[1] = (tmp>>8)&0xFF;
        ptr[2] = (tmp>>16)&0xFF;
        ptr[3] = (tmp>>24)&0xFF;
        break;
    case 4: //VBAN_BITFMT_32_FLOAT:
        tp = (uint32_t*)(&sample);
        tmp = *tp;
        ptr[0] = tmp&0xFF;
        ptr[1] = (tmp>>8)&0xFF;
        ptr[2] = (tmp>>16)&0xFF;
        ptr[3] = (tmp>>24)&0xFF;
        break;
    case 5: //VBAN_BITFMT_64_FLOAT:
    default:
        break;
    }
}


inline float convertSampleRX(uint8_t* ptr, uint8_t bit_fmt)
{
    int value = 0;

    switch (bit_fmt)
    {
        case 0: //VBAN_BITFMT_8_INT:
        return (float)(*((int8_t const*)ptr)) / (float)(1 << 7);

        case 1: //VBAN_BITFMT_16_INT:
        return (float)(*((int16_t const*)ptr)) / (float)(1 << 15);

        case 2: //VBAN_BITFMT_24_INT:
        value = (((int8_t)ptr[2])<<16) + (ptr[1]<<8) + ptr[0];
        return (float)(value) / (float)(1 << 23);

        case 3: //VBAN_BITFMT_32_INT:
        return (float)*((int32_t const*)ptr) / (float)(1 << 31);

        case 4: //VBAN_BITFMT_32_FLOAT:
        return *(float const*)ptr;

        //case 5: //VBAN_BITFMT_64_FLOAT:
        default:
        fprintf(stderr, "Convert Error! %d\n", bit_fmt);
        return 0.0;
    }
}


inline int getSampleRateIndex(long host_samplerate)
{
    int index = -1;
    uint8_t i;
    for (i=0; i<VBAN_SR_MAXNUMBER; i++) if (host_samplerate==VBanSRList[i]) index = i;
    return index;
}


inline uint stripPacket(uint8_t resolution, uint16_t nchannels)
{
    uint framesize = VBanBitResolutionSize[resolution]*nchannels;
    uint nframes = VBAN_DATA_MAX_SIZE/framesize;
    if (nframes>VBAN_SAMPLES_MAX_NB) nframes = VBAN_SAMPLES_MAX_NB;
    return nframes*framesize;
}


inline uint stripData(uint datasize, uint8_t resolution, uint16_t nchannels)
{
    uint framesize = VBanBitResolutionSize[resolution]*nchannels;
    uint nframes = datasize/framesize;
    if (nframes>VBAN_SAMPLES_MAX_NB) nframes = VBAN_SAMPLES_MAX_NB;
    return nframes*framesize;
}

inline uint calcNFrames(uint datasize, uint8_t resolution, uint16_t nchannels)
{
    uint framesize = VBanBitResolutionSize[resolution]*nchannels;
    uint nframes = datasize/framesize;
    return nframes;
}


inline uint pktToFloatBuf(uint pktlen, uint8_t resolution)
{
    return (pktlen/VBanBitResolutionSize[resolution])*4;
}


#ifdef __linux__

inline snd_pcm_format_t vban_to_alsa_format(enum VBanBitResolution bit_resolution)
{
    switch (bit_resolution)
    {
        case VBAN_BITFMT_8_INT:
            return SND_PCM_FORMAT_S8;

        case VBAN_BITFMT_16_INT:
            return SND_PCM_FORMAT_S16;

        case VBAN_BITFMT_24_INT:
            return SND_PCM_FORMAT_S24;

        case VBAN_BITFMT_32_INT:
            return SND_PCM_FORMAT_S32;

        case VBAN_BITFMT_32_FLOAT:
            return SND_PCM_FORMAT_FLOAT;

        case VBAN_BITFMT_64_FLOAT:
            return SND_PCM_FORMAT_FLOAT64;

        default:
            return SND_PCM_FORMAT_UNKNOWN;
    }
}

#endif

#endif
