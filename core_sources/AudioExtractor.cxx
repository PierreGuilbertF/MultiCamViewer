/*=========================================================================

  Program:   Visualization Toolkit
  Module:    AudioExtractor.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   AudioExtractor
 * @brief   Extract the audio from a video
 *
*/

#include "AudioExtractor.h"

void AudioExtract(std::string filename, const int sample_rate, double* data, int* size, int MaxNbr)
{
  // This registers all available file formats and codecs
  // with the library so theywill be used automatically
  // when a file with the corresponding format/codec is opened.
  av_register_all();

  // Read the file header and stores information about the
  // file format in the AVFormatContext structure we have given it.
  // The last three arguments are used to specify the file format,
  // buffer size, and format options, but by setting this to NULL or 0,
  // libavformat will auto-detect these
  AVFormatContext *format = avformat_alloc_context();
  if (avformat_open_input(&format, filename.c_str(), NULL, NULL) != 0)
  {
    std::cout << "AudioExtractor : Error opening the file" << std::endl;
    return; // Couldn't open file
  }

  // Retrieve stream information in the header of the container
  // This function populates pFormatCtx->streams with the proper information
  if(avformat_find_stream_info(format, NULL) < 0)
  {
    std::cout << "AudioExtractor : No streams found" << std::endl;
    return;
  }

  // Find the index of the first audio stream
  int stream_index =- 1;
  for (int i = 0; i < format->nb_streams; i++)
  {
    if (format->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO)
    {
        stream_index = i;
        break;
    }
  }
  if (stream_index == -1)
  {
    std::cout << "AudioExtractor : No audio streams found" << std::endl;
    return;
  }

  // Stream on an audio data
  AVStream* stream = format->streams[stream_index];

  // find & open codec
  AVCodecContext* codec = stream->codec;
  if (avcodec_open2(codec, avcodec_find_decoder(codec->codec_id), NULL) < 0)
  {
    std::cout << "AudioExtractor : Failed to open decoder for audio stream" << std::endl;
    return;
  }

  std::cout << "Filename : " << filename.c_str() << std::endl;
  std::cout << "Nbr channel : " << codec->channels << std::endl;
  std::cout << "Sample rate : " << codec->sample_rate << std::endl;

  // prepare resampler
  struct SwrContext* swr = swr_alloc();
  av_opt_set_int(swr, "in_channel_count",  codec->channels, 0);
  av_opt_set_int(swr, "out_channel_count", 1, 0);
  av_opt_set_int(swr, "in_channel_layout",  codec->channel_layout, 0);
  av_opt_set_int(swr, "out_channel_layout", AV_CH_LAYOUT_MONO, 0);
  av_opt_set_int(swr, "in_sample_rate", codec->sample_rate, 0);
  av_opt_set_int(swr, "out_sample_rate", sample_rate, 0);
  av_opt_set_sample_fmt(swr, "in_sample_fmt",  codec->sample_fmt, 0);
  av_opt_set_sample_fmt(swr, "out_sample_fmt", AV_SAMPLE_FMT_DBL,  0);
  swr_init(swr);
  if (!swr_is_initialized(swr))
  {
    std::cout << "AudioExtractor : Resampler has not been properly initialized" << std::endl;
    return;
  }

  // prepare to read data
  AVPacket packet; // piece of data, can contain one or more frame
  av_init_packet(&packet);
  AVFrame* frame = av_frame_alloc(); // a frame of data
  if (!frame)
  {
    std::cout << "AudioExtractor : Error allocating the frame" << std::endl;
    return;
  }

  // iterate through frames
  *data = NULL;
  *size = 0;

  while (av_read_frame(format, &packet) >= 0)
  {
    // decode one frame
    int gotFrame;
    if (avcodec_decode_audio4(codec, frame, &gotFrame, &packet) < 0)
    {
      break;
    }
    if (!gotFrame)
    {
      continue;
    }

    // resample frames
    double* buffer;
    av_samples_alloc((uint8_t**) &buffer, NULL, 1, frame->nb_samples, AV_SAMPLE_FMT_DBL, 0);
    int frame_count = swr_convert(swr, (uint8_t**) &buffer, frame->nb_samples, (const uint8_t**) frame->data, frame->nb_samples);

    for (int k = 0; k < frame_count; ++k)
    {
      data[size[0] + k] = static_cast<double>(buffer[k]);
      size[0]++;
      if (size[0] >= MaxNbr)
      {
        break;
      }
    }

    if (size[0] >= MaxNbr)
    {
      std::cout << "AudioExtractor : Max data limit reached" << std::endl;
      break;
    }
    //count = count + frame->nb_samples;
    // append resampled frames to data
    /**data = (double*) realloc(*data, (*size + frame->nb_samples) * sizeof(double));
    memcpy(*data + *size, buffer, frame_count * sizeof(double));
    *size += frame_count;*/
  }

  // clean up
  av_frame_free(&frame);
  swr_free(&swr);
  avcodec_close(codec);
  avformat_free_context(format);
}
