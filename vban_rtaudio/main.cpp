/*
 *  Copyright (c) 2023 by Billy the Hippo <billythehippo@yandex.ru>
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with vban.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <RtAudio.h>
//#include <rtaudio_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>

#include "vban.h"
#include "vban_functions.h"
#include "udpsocket.h"
#include "ringbuffer.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define CARD_NAME_LENGTH 64


class RtError : public std::exception
{
 public:
  //! Defined RtError types.
  enum Type {
    WARNING,           /*!< A non-critical error. */
    DEBUG_WARNING,     /*!< A non-critical error which might be useful for debugging. */
    UNSPECIFIED,       /*!< The default, unspecified error type. */
    NO_DEVICES_FOUND,  /*!< No devices found on system. */
    INVALID_DEVICE,    /*!< An invalid device ID was specified. */
    MEMORY_ERROR,      /*!< An error occured during memory allocation. */
    INVALID_PARAMETER, /*!< An invalid parameter was specified to a function. */
    INVALID_USE,       /*!< The function was called incorrectly. */
    DRIVER_ERROR,      /*!< A system driver error occured. */
    SYSTEM_ERROR,      /*!< A system error occured. */
    THREAD_ERROR       /*!< A thread error occured. */
  };

  //! The constructor.
  RtError( const std::string& message, Type type = RtError::UNSPECIFIED ) throw() : message_(message), type_(type) {}

  //! The destructor.
  virtual ~RtError( void ) throw() {}

  //! Prints thrown error message to stderr.
  virtual void printMessage( void ) const throw() { std::cerr << '\n' << message_ << "\n\n"; }

  //! Returns the thrown error message type.
  virtual const Type& getType(void) const throw() { return type_; }

  //! Returns the thrown error message string.
  virtual const std::string& getMessage(void) const throw() { return message_; }

  //! Returns the thrown error message as a c-style string.
  virtual const char* what( void ) const throw() { return message_.c_str(); }

 protected:
  std::string message_;
  Type type_;
};


typedef struct
{
    uint32_t samplerate;
    uint32_t nframes;
    uint8_t nbchannels;
    uint8_t redundancy;
    uint8_t format;
    uint8_t format_vban;
    char ipaddr[16];
    uint32_t ip;
    uint32_t iprx;
    uint16_t port;
    uint8_t streamDirection; // 1 is capture, 0 is playback
    uint8_t hwnchannels;
    uint8_t apiindex;
    char pipename[64];
    VBanPacket packet;
    VBanHeader header;
    uint16_t packetdatalen;
    uint32_t packetnum;
    int pipedesc;
    int vban_sd;
    struct sockaddr_in vban_si;
    struct sockaddr_in vban_si_other;
    struct pollfd polldesc;
    char cardname[CARD_NAME_LENGTH];
    jack_ringbuffer_t* ringbuffer;
    size_t ringbuffersize;
    char* rxbuf;
    int32_t rxbuflen;
} streamConfig_t;


uint8_t format_vban_to_rtaudio(uint8_t vban_format)
{
    switch (vban_format)
    {
    case VBAN_BITFMT_8_INT:
        return RTAUDIO_SINT8;
    case VBAN_BITFMT_16_INT:
        return RTAUDIO_SINT16;
    case VBAN_BITFMT_24_INT:
        return RTAUDIO_SINT24;
    case VBAN_BITFMT_32_INT:
        return RTAUDIO_SINT32;
    case VBAN_BITFMT_32_FLOAT:
        return RTAUDIO_FLOAT32;
    case VBAN_BITFMT_64_FLOAT:
        return RTAUDIO_FLOAT64;
    default:
        fprintf (stderr, "Warning! Unsupported format, will be converted to S16\n");
        return RTAUDIO_SINT16;
    }
}


void* rxThread(void* param)
{
    streamConfig_t* stream = (streamConfig_t*)param;
    int packetlen;
    char* dstptr;
    char* srcptr;
    float* srcfptr;
    int32_t nframes;
    int32_t frame;
    int32_t nchannels = stream->header.format_nbc + 1;
    int32_t channel;
    size_t framesize = VBanBitResolutionSize[stream->header.format_bit&VBAN_BIT_RESOLUTION_MASK]*nchannels;
    int datalen;
    int32_t tmp;
    char* samplebuffer = (char*)malloc(framesize);

    while (1)
    {
        while (poll(&stream->polldesc, 1, 500))
        {
            if (stream->port)
            {
                packetlen = UDP_recv(stream->vban_sd, &stream->vban_si, (uint8_t*)&stream->packet, VBAN_PROTOCOL_MAX_SIZE);
                datalen = packetlen - VBAN_HEADER_SIZE;
            }
            else
            {
                packetlen = read(stream->polldesc.fd, &stream->packet.header, VBAN_HEADER_SIZE);
            }
            if (packetlen)
            {
                stream->iprx = stream->vban_si.sin_addr.s_addr; // get senrder's ip
                if (stream->packet.header.vban==VBAN_HEADER_FOURC) // check for VBAN packet
                {
                    switch (stream->packet.header.format_SR&VBAN_PROTOCOL_MASK)
                    {
                    case VBAN_PROTOCOL_AUDIO:
                        if (stream->port)
                        {
                            if (stream->ip == 0) // still not started
                            {
                                if (stream->packet.header.format_SR!=stream->header.format_SR)
                                {
                                    fprintf(stderr, "Error: Samplerate mismatch!");
                                    free(samplebuffer);
                                    exit(1);
                                    //return 1;
                                    // TODO : REWORK TO RESAMPLER!
                                }
                                if ((strncmp(stream->packet.header.streamname, stream->header.streamname, VBAN_STREAM_NAME_SIZE)==0)&&(stream->packet.header.format_nbc==stream->header.format_nbc)) stream->ip = stream->iprx;
                            }
                        }
                        else
                        {
                            datalen = VBanBitResolutionSize[stream->packet.header.format_bit&VBAN_BIT_RESOLUTION_MASK]*(stream->packet.header.format_nbc+1)*(stream->packet.header.format_nbs+1);
                            if (datalen==read(stream->polldesc.fd, stream->packet.data, datalen))
                            {
                                packetlen+= datalen;
                            }
                        }

                        if ((strncmp(stream->packet.header.streamname, stream->header.streamname, VBAN_STREAM_NAME_SIZE)==0)&&(stream->ip == stream->iprx)&&
                                (stream->packet.header.format_SR  == stream->header.format_SR)&&
                                (stream->packet.header.format_nbc == stream->header.format_nbc)&&
                                (stream->packet.header.nuFrame    != stream->header.nuFrame))
                        {
                            nframes = ((uint32_t)stream->packet.header.format_nbs+1);
                            srcptr = &((char*)&stream->packet)[VBAN_HEADER_SIZE];

                            if ((stream->packet.header.format_bit&VBAN_BIT_RESOLUTION_MASK)==(stream->header.format_bit&VBAN_BIT_RESOLUTION_MASK))
                            {
                                for (frame=0; frame<nframes; frame++)
                                {
                                    if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, srcptr, framesize);
                                    srcptr+= framesize;
                                }
                            }
                            else
                            {
                                switch (stream->packet.header.format_bit&VBAN_BIT_RESOLUTION_MASK)
                                {
                                case VBAN_BITFMT_16_INT:
                                    switch (stream->header.format_bit&VBAN_BIT_RESOLUTION_MASK)
                                    {
                                    case VBAN_BITFMT_24_INT:
                                        for (frame=0; frame<nframes; frame++)
                                        {
                                            dstptr = samplebuffer;
                                            for (channel=0; channel<nchannels; channel++)
                                            {
                                                dstptr[0] = 0;
                                                dstptr[1] = srcptr[0];
                                                dstptr[2] = srcptr[1];
                                                srcptr+= 2; // 16 bit = 2 bytes
                                                dstptr+= 3; // 24 bit = 3 bytes
                                            }
                                            if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, samplebuffer, framesize);                                        }
                                        break;
                                    case VBAN_BITFMT_32_FLOAT:
                                        for (frame=0; frame<nframes; frame++)
                                        {
                                            dstptr = samplebuffer;
                                            for (channel=0; channel<nchannels; channel++)
                                            {
                                                ((float*)dstptr)[0] = (float)(*((int16_t const*)srcptr))/(float)(1 << 15);
                                                srcptr+= 2; // 16 bit = 2 bytes
                                                dstptr+= 4; // 32 bit = 4 bytes
                                            }
                                            if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, samplebuffer, framesize);
                                        }
                                        break;
                                    default:
                                        break;
                                    }
                                    break;
                                case VBAN_BITFMT_24_INT:
                                    switch (stream->header.format_bit&VBAN_BIT_RESOLUTION_MASK)
                                    {
                                    case VBAN_BITFMT_16_INT:
                                        for (frame=0; frame<nframes; frame++)
                                        {
                                            dstptr = samplebuffer;
                                            for (channel=0; channel<nchannels; channel++)
                                            {
                                                dstptr[0] = srcptr[1];
                                                dstptr[1] = srcptr[2];
                                                srcptr+= 3; // 24 bit = 3 bytes
                                                dstptr+= 2; // 16 bit = 2 bytes
                                            }
                                            if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, samplebuffer, framesize);
                                        }
                                        break;
                                    case VBAN_BITFMT_32_FLOAT:
                                        for (frame=0; frame<nframes; frame++)
                                        {
                                            dstptr = samplebuffer;
                                            for (channel=0; channel<nchannels; channel++)
                                            {
                                                ((float*)dstptr)[0] = (float)(((srcptr[2])<<16) + ((uint8_t)srcptr[1]<<8) + (uint8_t)srcptr[0])/(float)(1 << 23);
                                                srcptr+= 3; // 24 bit = 3 bytes
                                                dstptr+= 4; // 32 bit = 4 bytes
                                            }
                                            if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, samplebuffer, framesize);
                                        }
                                        break;
                                    default:
                                        break;
                                    }
                                    break;
                                case VBAN_BITFMT_32_FLOAT:
                                    srcfptr = (float*)&stream->packet.data;
                                    switch (stream->header.format_bit&VBAN_BIT_RESOLUTION_MASK)
                                    {
                                    case VBAN_BITFMT_16_INT:
                                        for (frame=0; frame<nframes; frame++)
                                        {
                                            dstptr = samplebuffer;
                                            for (channel=0; channel<nchannels; channel++)
                                            {
                                                tmp = (int16_t)roundf(32767.0f*srcfptr[0]);
                                                dstptr[0] = tmp&0xFF;
                                                dstptr[1] = (tmp>>8)&0xFF;
                                                dstptr+= 2; // 16 bit = 2 bytes
                                                srcptr+= 4; // 32 bit = 4 bytes
                                                srcfptr++;
                                            }
                                            if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, samplebuffer, framesize);
                                        }
                                        break;
                                    case VBAN_BITFMT_24_INT:
                                        for (frame=0; frame<nframes; frame++)
                                        {
                                            dstptr = samplebuffer;
                                            for (channel=0; channel<nchannels; channel++)
                                            {
                                                tmp = (int32_t)roundf(8388607.0f*srcfptr[0]);
                                                dstptr[0] = tmp&0xFF;
                                                dstptr[1] = (tmp>>8)&0xFF;
                                                dstptr[2] = (tmp>>16)&0xFF;
                                                dstptr+= 3; // 24 bit = 3 bytes
                                                srcptr+= 4; // 32 bit = 4 bytes
                                                srcfptr++;
                                            }
                                            if (jack_ringbuffer_write_space(stream->ringbuffer)>=framesize) jack_ringbuffer_write(stream->ringbuffer, samplebuffer, framesize);
                                        }
                                        break;
                                    default:
                                        break;
                                    }
                                    break;
                                default:
                                    break;
                                }
                            }

                            stream->header.nuFrame = stream->packet.header.nuFrame;
                        }
                        break;
                    case VBAN_PROTOCOL_SERIAL:
                        break;
                    case VBAN_PROTOCOL_TXT:
                        break;
                    case VBAN_PROTOCOL_USER:
                        break;
                    default:
                        break;
                    }
                }
            }
        }
        fprintf(stderr, "500 ms no signal!\n");
        if (stream->port)
        {
            stream->iprx = 0;
            while (poll(&stream->polldesc, 1, 0))
            {
                packetlen = UDP_recv(stream->vban_sd, &stream->vban_si, NULL, VBAN_PROTOCOL_MAX_SIZE);
            }
        }//*/
    }
}


static int rtaudio_callback(void *outbuf, void *inbuf, unsigned int nFrames, double streamtime, RtAudioStreamStatus	status, void *userdata)
{
    (void)inbuf;
    char* buf = nullptr;
    streamConfig_t* stream = (streamConfig_t*)userdata;
    uint32_t nframes = stream->nframes;
    uint32_t frame;
    uint32_t nbchannels = stream->nbchannels;
    uint32_t channel;
    uint32_t samplesize = 2;
    uint32_t framesize = samplesize*nbchannels;
    uint32_t buflen = nFrames*framesize;
    char* ptr;
    char* dstptr;
    uint32_t pac;
    jack_ringbuffer_t* ringbuf = stream->ringbuffer;

    if (stream->streamDirection == 1) // CAPTURE
    {
        buf = (char*)inbuf;
        ptr = buf;
        samplesize = VBanBitResolutionSize[stream->packet.header.format_bit&VBAN_BIT_RESOLUTION_MASK];
        framesize = samplesize*nbchannels;
        buflen = nFrames*framesize;

        if ((stream->nbchannels!=stream->hwnchannels)) // pulseaudio &&((stream->apiindex==2))
        {
            if ((stream->nbchannels==2)&&(stream->hwnchannels==1))
            {
                for (pac=0; pac<stream->packetnum; pac++)
                {
                    dstptr = stream->packet.data;
                    for (frame=0; frame<nframes; frame++)
                    {
                        memcpy(dstptr, ptr, samplesize);
                        dstptr+=samplesize;
                        memcpy(dstptr, ptr, samplesize);
                        dstptr+=samplesize;
                        ptr+= samplesize;
                    }
                    if(stream->port) UDP_send(stream->vban_sd, &stream->vban_si_other, (uint8_t*)&stream->packet, stream->packetdatalen+VBAN_HEADER_SIZE);
                    else
                    {
                        write(stream->pipedesc, (char*)&stream->packet, stream->packetdatalen+VBAN_HEADER_SIZE);
                        if (stream->pipedesc==STDOUT_FILENO) fflush(stdout);
                    }
                    stream->packet.header.nuFrame++;
                }
            }
        }
        else
        {
            samplesize = VBanBitResolutionSize[stream->packet.header.format_bit&VBAN_BIT_RESOLUTION_MASK];
            framesize = samplesize*nbchannels;
            buflen = nFrames*framesize;

            for (pac=0; pac<stream->packetnum; pac++)
            {
                memcpy(stream->packet.data, ptr, (framesize*(stream->packet.header.format_nbs+1)));
                if(stream->port) UDP_send(stream->vban_sd, &stream->vban_si_other, (uint8_t*)&stream->packet, stream->packetdatalen+VBAN_HEADER_SIZE);
                else
                {
                    write(stream->pipedesc, (char*)&stream->packet, stream->packetdatalen+VBAN_HEADER_SIZE);
                    //if (stream->pipedesc==STDOUT_FILENO)
                        fflush(stdout);
                }
                ptr+= stream->packetdatalen;//(framesize*(stream->packet.header.format_nbs+1));
                stream->packet.header.nuFrame++;
            }
        }
    }
    else // PLAYBACK
    {
        buf = (char*)outbuf;
        ptr = stream->rxbuf;

        for (frame=0; frame<nframes; frame++)
        {
            if (jack_ringbuffer_read_space(ringbuf)>=framesize) jack_ringbuffer_read(ringbuf, ptr, framesize);
            ptr+= framesize;
        }
        memcpy(buf, stream->rxbuf, buflen);
    }

    return 0;
}


void help(void)
{
    fprintf(stderr, "DEMO VBAN RtAudio client for network and pipes/fifos\n\nBy Billy the Hippo\n\nusage: vban_rtaudio <args>\n\n");
    fprintf(stderr, "-m - mode: rx - net/fifo to audio, tx (default) - audio to net/fifo\n");
    fprintf(stderr, "-d - device name (for ALSA without \"hw:\", for others - see manuals)\n");
    fprintf(stderr, "-s - samplerate (default 48000)\n");
    fprintf(stderr, "-q - quantum, buffer size (Attention!!! Default is 128!!! Made for musicians.)\n");
    fprintf(stderr, "-c - number of channels\n");
    fprintf(stderr, "-n - redundancy 1 to 10 (in rx mode - as \"net quality\", it tx mode not implemented yet)\n");
    fprintf(stderr, "-f - format: 16, 24, 32f\n");
    fprintf(stderr, "-i - ip address or pipe name\n");
    fprintf(stderr, "-p - ip port (if 0 - pipe)\n");
    fprintf(stderr, "-s - Stream name, up to symbols\n");
    fprintf(stderr, "-b - backend (alsa, pulse, jack for linux; coreaudio, jack for mac; asio, wasapi for windows)\n");
    fprintf(stderr, "-h - show this help\n");
    exit(0);
}


int get_options(streamConfig_t* stream, int argc, char *argv[])
{
    int index;
    char c;
    static const struct option options[] =
    {
        {"mode",        required_argument,  0, 'm'},
        {"device",      required_argument,  0, 'd'},
        {"samplerate",  required_argument,  0, 'r'},
        {"bufsize",     required_argument,  0, 'q'},
        {"nbchannels",  required_argument,  0, 'c'},
        {"redundancy",  required_argument,  0, 'n'},
        {"format",      required_argument,  0, 'f'},
        {"ipaddr",      required_argument,  0, 'i'},
        {"port",        required_argument,  0, 'p'},
        {"streamname",  required_argument,  0, 's'},
        {"backend",     required_argument,  0, 'b'},
        {"help",        no_argument,        0, 'h'},
        {0,             0,                  0,  0 }
    };
    int ipnums[4];

    c = getopt_long(argc, argv, "m:d:r:q:c:n:f:i:p:s:b:h", options, 0);
    if (c==-1)
    {
        help();
        return 0;
    }
    while(c!=-1)
    {
        switch (c)
        {
        case 'h':
            help();
            exit(0);
        case 'm':
            if ((optarg[1]=='1')||(strncmp(optarg, "TX", 2)==0)||(strncmp(optarg, "tx", 2)==0)) stream->streamDirection = 1;
            break;
        case 'd':
            if (optarg[0]==0)
            {
                fprintf(stderr, "No audio device selected!\n");
                return 1;
            }
            else
            {
                memset(stream->cardname, 0, CARD_NAME_LENGTH);
                if (strlen(optarg)<CARD_NAME_LENGTH) strcpy(stream->cardname, optarg);
                else strncpy(stream->cardname, optarg, CARD_NAME_LENGTH-1);
            }
            break;
        case 'r':
            stream->samplerate = atoi(optarg);
            break;
        case 'q':
            stream->nframes = atoi(optarg);
            break;
        case 'c':
            stream->nbchannels = atoi(optarg);
            //packet.header.format_nbc = captureHandle.nbchannels - 1;
            break;
        case 'n':
            stream->redundancy = atoi(optarg);
            break;
        case 'f':
            if ((strncmp(optarg, "16I", 3)==0)|(strncmp(optarg, "16i", 3)==0)) stream->format_vban = VBAN_BITFMT_16_INT;
            else if ((strncmp(optarg, "24I", 3)==0)|(strncmp(optarg, "24i", 3)==0)) stream->format_vban = VBAN_BITFMT_24_INT;
            else if ((strncmp(optarg, "32F", 3)==0)|(strncmp(optarg, "32F", 3)==0)) stream->format_vban = VBAN_BITFMT_32_FLOAT;
            break;
        case 'i':
            memset(stream->ipaddr, 0, 16);
            if (strlen(optarg)<16) strcpy(stream->ipaddr, optarg);
            else strncpy(stream->ipaddr, optarg, 16);
            if(strncmp(stream->ipaddr, "pipe", 4))
            {
                sscanf(stream->ipaddr, "%d.%d.%d.%d", &ipnums[0], &ipnums[1], &ipnums[2], &ipnums[3]);
                if ((ipnums[0]<0)|(ipnums[1]<0)|(ipnums[2]<0)|(ipnums[3]<0)|(ipnums[0]>255)|(ipnums[1]>255)|(ipnums[2]>255)|(ipnums[3]>255))
                {
                    fprintf(stderr, "Error: incorrect IP address!\n");
                    return 1;
                }
                else
                {
                    for(index=0; index<4; index++) ((uint8_t*)&stream->ip)[index] = (uint8_t)ipnums[index];
                    if (stream->port==0) stream->port = 6980;
                }
            }
            else fprintf(stderr, "Using pipe!\n");
            break;
        case 'p':
            if(strncmp(stream->ipaddr, "pipe", 4)) stream->port = atoi(optarg);
            else
            {
                memset(stream->pipename, 0, 64);
                if (strlen(optarg)<64) strcpy(stream->pipename, optarg);
                else strncpy(stream->pipename, optarg, 64);
            }
            break;
        case 's':
            if (optarg[0]==0)
            {
                fprintf(stderr, "No stream name given!\n");
                return 1;
            }
            else
            {
                if (stream->streamDirection) // CAPTURE
                {
                    memset(stream->packet.header.streamname, 0, VBAN_STREAM_NAME_SIZE);
                    if (strlen(optarg)<VBAN_STREAM_NAME_SIZE) strcpy(stream->packet.header.streamname, optarg);
                    else strncpy(stream->packet.header.streamname, optarg, VBAN_STREAM_NAME_SIZE-1);
                }
                else // PLAYBACK
                {
                    memset(stream->header.streamname, 0, VBAN_STREAM_NAME_SIZE);
                    if (strlen(optarg)<VBAN_STREAM_NAME_SIZE) strcpy(stream->header.streamname, optarg);
                    else strncpy(stream->header.streamname, optarg, VBAN_STREAM_NAME_SIZE-1);
                }
            }
            break;
        case 'b':
            if ((strncmp(optarg, "alsa", 4)==0)||(strncmp(optarg, "ALSA", 4)==0))
            {
                stream->apiindex = 1;
            }
            else if ((strncmp(optarg, "pulse", 5)==0)||(strncmp(optarg, "PULSE", 5)==0))
            {
                stream->apiindex = 2;
            }
            else if ((strncmp(optarg, "jack", 4)==0)||(strncmp(optarg, "JACK", 4)==0))
            {
                stream->apiindex = 4;
            }
            else if ((strncmp(optarg, "coreaudio", 9)==0)||(strncmp(optarg, "COREAUDIO", 9)==0))
            {
                stream->apiindex = 5;
            }
            else if ((strncmp(optarg, "wasapi", 6)==0)||(strncmp(optarg, "WASAPI", 6)==0))
            {
                stream->apiindex = 6;
            }
            else if ((strncmp(optarg, "asio", 4)==0)||(strncmp(optarg, "ASIO", 4)==0))
            {
                stream->apiindex = 7;
            }
        }
        c = getopt_long(argc, argv, "m:d:r:q:c:n:f:i:p:s:b:h", options, 0);
    }
    return 0;
}


int main(int argc, char *argv[])
{
    RtAudio *audio;
    RtAudio::DeviceInfo devinfo;
    RtAudio::StreamParameters* param;
    uint8_t devId = 0;
    streamConfig_t stream;

    pthread_t rxtid;
    pthread_attr_t attr;
    pthread_mutex_t read_thread_lock = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t  data_ready = PTHREAD_COND_INITIALIZER;

    memset(&stream, 0, sizeof(streamConfig_t));
#if defined WIN32
    stream.apiindex = 7; // ASIO
#else
    stream.apiindex = 4; // JACK
#endif
    stream.streamDirection = 0;
    stream.nbchannels = 2;
    stream.samplerate = 48000;
    stream.nframes = 256;
    stream.format_vban = 1;

    uint32_t buflen;

    strcpy(stream.packet.header.streamname, "Stream1");
    strcpy(stream.ipaddr, "pipe");
    stream.port = 0;

    strcpy(stream.cardname, "default");

    get_options(&stream, argc, argv);

    if (stream.streamDirection==1)
    {
        strcpy(stream.pipename, "stdout");
        stream.pipedesc = 1;
    }
    else
    {
        strcpy(stream.pipename, "stdin");
        stream.pipedesc = 0;
    }

    stream.format = format_vban_to_rtaudio(stream.format_vban);

    try
    {
        switch (stream.apiindex)
        {
#ifdef __linux__
        case 1:
            audio = new RtAudio(RtAudio::LINUX_ALSA);
            break;
        case 2:
            audio = new RtAudio(RtAudio::LINUX_PULSE);
            break;
        case 3:
            audio = new RtAudio(RtAudio::LINUX_OSS);
            break;
#endif
        case 4:
            audio = new RtAudio(RtAudio::UNIX_JACK);
            break;
#ifdef __APPLE__
        case 5:
            audio = new RtAudio(RtAudio::MACOSX_CORE);
            break;
#endif
#if defined _WIN32 || defined _WIN64
        case 6:
            audio = new RtAudio(RtAudio::WINDOWS_WASAPI);
            break;
        case 7:
            audio = new RtAudio(RtAudio::WINDOWS_ASIO);
            break;
        case 8:
            audio = new RtAudio(RtAudio::WINDOWS_DS);
            break;
#endif
        case 9:
            audio = new RtAudio(RtAudio::RTAUDIO_DUMMY);
            break;
        default:
            fprintf(stderr, "Wrong audio backend!\n");
            exit(1);
        }

    }
    catch (RtError e)
    {
        fprintf(stderr, "fail to create RtAudio: %s¥n", e.what());
        return 1;
    }
    if (!audio)
    {
        fprintf(stderr, "fail to allocate RtAudio¥n");
        return 1;
    }

    /* probe audio devices */
    uint8_t devCnt = audio->getDeviceCount();
    uint8_t i = 0;
    char* sptr;
    //int num = atoi(stream.cardname);

    if (((stream.cardname[0]==0)||(strncmp(stream.cardname, "default", 7)==0))&&(stream.apiindex==2)) // PULSE
    {
        if (stream.streamDirection==1) devId = audio->getDefaultInputDevice();
        else devId = audio->getDefaultOutputDevice();
    }
    else for (i=0; i<devCnt; i++)
    {
        devinfo = audio->getDeviceInfo(i);
        sptr = strstr(&devinfo.name[0], stream.cardname);
        if (sptr!=NULL)
        {
            if (((stream.streamDirection==1)&&(devinfo.inputChannels!=0))||((stream.streamDirection==0)&&(devinfo.outputChannels!=0)))
            {
                devId = i;
                break;
            }
        }
    }//*/
    /*if (stream.streamDirection==1) devId = audio->getDefaultInputDevice();
    else devId = audio->getDefaultOutputDevice();//*/
    if (i==devCnt)
    {
        fprintf(stderr, "No devices found!\n");
        exit(1);
    }

    /* Setup stream parameters */
    param = new RtAudio::StreamParameters();
    param->deviceId = devId;
    param->nChannels = stream.nbchannels;

    devinfo = audio->getDeviceInfo(devId);

    if (stream.streamDirection==1)
    {
        if (devinfo.inputChannels<param->nChannels) param->nChannels = devinfo.inputChannels;
        audio->openStream(NULL, param, stream.format, stream.samplerate, &stream.nframes, rtaudio_callback, &stream);
    }
    else
    {
        if (devinfo.outputChannels<param->nChannels) param->nChannels = devinfo.outputChannels;
        audio->openStream(param, NULL, stream.format, stream.samplerate, &stream.nframes, rtaudio_callback, &stream);
    }

    stream.hwnchannels = param->nChannels;
    buflen = stream.nbchannels*stream.nframes*VBanBitResolutionSize[stream.format_vban];

    if (stream.streamDirection==1)
    {
        stream.packetdatalen = buflen;
        stream.packetnum = 1;
        while(stream.packetdatalen>VBAN_DATA_MAX_SIZE)
        {
            stream.packetdatalen = stream.packetdatalen>>1;
            stream.packetnum = stream.packetnum<<1;
        }

        stream.packet.header.vban = VBAN_HEADER_FOURC;
        stream.packet.header.format_SR = getSampleRateIndex(stream.samplerate);
        stream.packet.header.format_bit = stream.format_vban;
        stream.packet.header.format_nbs = buflen/(stream.nbchannels*VBanBitResolutionSize[stream.format_vban]) - 1;
        stream.packet.header.format_nbc = stream.nbchannels - 1;

        if (strncmp(stream.ipaddr, "pipe", 4)) // using socket
        {
            stream.vban_sd = UDP_init(&stream.vban_sd, &stream.vban_si, &stream.vban_si_other, stream.ipaddr, stream.port, 'c', 1, 6);
            if (stream.vban_sd<0)
            {
                fprintf(stderr, "Can't bind the socket! Maybe, port is busy?\n");
                return 1;
            }
            printf("Socket is successfully created! Port: %d, priority: 6\n", stream.port);
        }
        else // using pipe
        {
            stream.port = 0;
            if (strncmp(stream.pipename, "stdout", 6)) // named pipe
            {
                stream.pipedesc = open(stream.pipename, O_WRONLY);
                mkfifo(stream.pipename, 0666);
            }
            else stream.pipedesc = 1; // stdout
        }
    }
    else
    {
        stream.header.vban = VBAN_HEADER_FOURC;
        stream.header.format_SR = getSampleRateIndex(stream.samplerate);
        stream.header.format_bit = stream.format_vban;
        stream.header.format_nbs = 0;
        stream.header.format_nbc = stream.nbchannels - 1;

        stream.rxbuflen = 2*buflen;
        stream.rxbuf = (char*)malloc(stream.rxbuflen);
        memset(stream.rxbuf, 0, stream.rxbuflen);
        stream.ringbuffersize = stream.rxbuflen*(stream.redundancy+1);
        stream.ringbuffer = jack_ringbuffer_create(stream.ringbuffersize);

        char* const zeros = (char*)calloc(1, stream.ringbuffersize/2);
        jack_ringbuffer_write(stream.ringbuffer, zeros, stream.ringbuffersize/2);
        free(zeros);//*/

        if (strncmp(stream.ipaddr, "pipe", 4)) // using socket
        {
            stream.vban_sd = UDP_init(&stream.vban_sd, &stream.vban_si, &stream.vban_si_other, stream.ipaddr, stream.port, 's', 1, 6);
            if (stream.vban_sd<0)
            {
                fprintf(stderr, "Can't bind the socket! Maybe, port is busy?\n");
                return 1;
            }
            printf("Socket is successfully created! Port: %d, priority: 6\n", stream.port);
            stream.polldesc.fd = stream.vban_sd;
            stream.polldesc.events = POLLIN;

            while (poll(&stream.polldesc, 1, 0)) UDP_recv(stream.vban_sd, &stream.vban_si, NULL, VBAN_PROTOCOL_MAX_SIZE); //flush socket
        }
        else // using pipe
        {
            stream.port = 0;
            if (strncmp(stream.pipename, "stdin", 6)) // named pipe
            {
                stream.pipedesc = open(stream.pipename, O_RDONLY);
                mkfifo(stream.pipename, 0666);
            }
            else stream.pipedesc = 0; // stdin
            stream.polldesc.fd = stream.pipedesc;
            stream.polldesc.events = POLLIN;
        }
        pthread_attr_init(&attr);
        pthread_create(&rxtid, &attr, rxThread, (void*)&stream);
    }

    audio->startStream();

    while(1) sleep(1);

    audio->stopStream();
    audio->closeStream();
    delete audio;

    return 0;
}

#if defined(__cplusplus)
}
#endif
