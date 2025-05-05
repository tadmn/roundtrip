#pragma once

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <numbers>
#include <span>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/channel_layout.h>
#include <libswresample/swresample.h>
}

class MP3Effect {
public:
    // Configure the MP3 effect with sample rate, channels, and bitrate (in kbps)
    MP3Effect(int sampleRate, int channels, int bitrate = 128)
        : mSampleRate(sampleRate)
        , mChannels(channels)
        , mBitrate(bitrate) {
        Initialize();
    }

    ~MP3Effect() {
        Cleanup();
    }

    // Process PCM audio through MP3 encoding/decoding
    std::vector<float> Process(std::span<const float> inputPcm) {
        if (!mIsInitialized) {
            throw std::runtime_error("MP3Effect not properly initialized");
        }

        // Encode PCM to MP3
        auto mp3Data = EncodePcmToMp3(inputPcm);

        // Decode MP3 back to PCM
        return DecodeMp3ToPcm(mp3Data);
    }

    // Change the MP3 bitrate (in kbps)
    void SetBitrate(int bitrate) {
        if (mBitrate == bitrate) return;

        mBitrate = bitrate;
        Cleanup();
        Initialize();
    }

private:
    // Audio configuration
    int mSampleRate;
    int mChannels;
    int mBitrate;

    // FFmpeg context objects
    AVCodecContext* mEncodeContext = nullptr;
    AVCodecContext* mDecodeContext = nullptr;
    SwrContext* mEncodeResampler = nullptr;
    SwrContext* mDecodeResampler = nullptr;

    bool mIsInitialized = false;

    void Initialize() {
        // Set up encoder
        const AVCodec* encoder = avcodec_find_encoder(AV_CODEC_ID_MP3);
        if (!encoder) {
            throw std::runtime_error("Could not find MP3 encoder");
        }

        mEncodeContext = avcodec_alloc_context3(encoder);
        if (!mEncodeContext) {
            throw std::runtime_error("Could not allocate encoder context");
        }

        mEncodeContext->bit_rate = mBitrate * 1'000;
        mEncodeContext->sample_rate = mSampleRate;

        // Set channel layout using the new API
        av_channel_layout_default(&mEncodeContext->ch_layout, mChannels);
        if (mChannels == 2) {
            av_channel_layout_from_string(&mEncodeContext->ch_layout, "stereo");
        }

        mEncodeContext->sample_fmt = AV_SAMPLE_FMT_S16P;  // MP3 encoder uses planar signed 16-bit

        if (avcodec_open2(mEncodeContext, encoder, nullptr) < 0) {
            Cleanup();
            throw std::runtime_error("Could not open encoder");
        }

        // Set up decoder
        const AVCodec* decoder = avcodec_find_decoder(AV_CODEC_ID_MP3);
        if (!decoder) {
            Cleanup();
            throw std::runtime_error("Could not find MP3 decoder");
        }

        mDecodeContext = avcodec_alloc_context3(decoder);
        if (!mDecodeContext) {
            Cleanup();
            throw std::runtime_error("Could not allocate decoder context");
        }

        mDecodeContext->bit_rate = mBitrate * 1'000;
        mDecodeContext->sample_rate = mSampleRate;

        // Set channel layout using the new API
        av_channel_layout_default(&mDecodeContext->ch_layout, mChannels);
        if (mChannels == 2) {
            av_channel_layout_from_string(&mDecodeContext->ch_layout, "stereo");
        }

        mDecodeContext->sample_fmt = AV_SAMPLE_FMT_S16P;

        if (avcodec_open2(mDecodeContext, decoder, nullptr) < 0) {
            Cleanup();
            throw std::runtime_error("Could not open decoder");
        }

        // Set up resamplers for format conversion
        mEncodeResampler = swr_alloc();
        if (!mEncodeResampler) {
            Cleanup();
            throw std::runtime_error("Could not allocate encode resampler");
        }

        // Create channel layouts for resampler
        AVChannelLayout stereoLayout = {.order = AV_CHANNEL_ORDER_UNSPEC, .nb_channels = 0};
        av_channel_layout_from_string(&stereoLayout, "stereo");

        // Set channel layouts using the new API
        av_opt_set_chlayout(mEncodeResampler, "in_chlayout", &stereoLayout, 0);
        av_opt_set_chlayout(mEncodeResampler, "out_chlayout", &stereoLayout, 0);

        av_opt_set_int(mEncodeResampler, "in_sample_rate", mSampleRate, 0);
        av_opt_set_int(mEncodeResampler, "out_sample_rate", mSampleRate, 0);
        av_opt_set_sample_fmt(mEncodeResampler, "in_sample_fmt", AV_SAMPLE_FMT_FLT, 0);
        av_opt_set_sample_fmt(mEncodeResampler, "out_sample_fmt", AV_SAMPLE_FMT_S16P, 0);

        if (swr_init(mEncodeResampler) < 0) {
            // Free the temporary channel layout
            av_channel_layout_uninit(&stereoLayout);
            Cleanup();
            throw std::runtime_error("Could not initialize encode resampler");
        }

        mDecodeResampler = swr_alloc();
        if (!mDecodeResampler) {
            // Free the temporary channel layout
            av_channel_layout_uninit(&stereoLayout);
            Cleanup();
            throw std::runtime_error("Could not allocate decode resampler");
        }

        // Set channel layouts using the new API
        av_opt_set_chlayout(mDecodeResampler, "in_chlayout", &stereoLayout, 0);
        av_opt_set_chlayout(mDecodeResampler, "out_chlayout", &stereoLayout, 0);

        av_opt_set_int(mDecodeResampler, "in_sample_rate", mSampleRate, 0);
        av_opt_set_int(mDecodeResampler, "out_sample_rate", mSampleRate, 0);
        av_opt_set_sample_fmt(mDecodeResampler, "in_sample_fmt", AV_SAMPLE_FMT_S16P, 0);
        av_opt_set_sample_fmt(mDecodeResampler, "out_sample_fmt", AV_SAMPLE_FMT_FLT, 0);

        // Free the temporary channel layout
        av_channel_layout_uninit(&stereoLayout);

        if (swr_init(mDecodeResampler) < 0) {
            Cleanup();
            throw std::runtime_error("Could not initialize decode resampler");
        }

        mIsInitialized = true;
    }

    void Cleanup() {
        if (mEncodeContext) {
            avcodec_free_context(&mEncodeContext);
            mEncodeContext = nullptr;
        }

        if (mDecodeContext) {
            avcodec_free_context(&mDecodeContext);
            mDecodeContext = nullptr;
        }

        if (mEncodeResampler) {
            swr_free(&mEncodeResampler);
            mEncodeResampler = nullptr;
        }

        if (mDecodeResampler) {
            swr_free(&mDecodeResampler);
            mDecodeResampler = nullptr;
        }

        mIsInitialized = false;
    }

    std::vector<uint8_t> EncodePcmToMp3(std::span<const float> inputPcm) {
        std::vector<uint8_t> mp3Data;

        // Create frame for input
        AVFrame* frame = av_frame_alloc();
        frame->format = AV_SAMPLE_FMT_S16P;

        // Set up channel layout for the frame
        av_channel_layout_from_string(&frame->ch_layout, "stereo");

        frame->sample_rate = mSampleRate;
        frame->nb_samples = 1152;  // MP3 frame size

        if (av_frame_get_buffer(frame, 0) < 0) {
            av_frame_free(&frame);
            throw std::runtime_error("Could not allocate audio frame");
        }

        // Create packet for output
        AVPacket* pkt = av_packet_alloc();
        if (!pkt) {
            av_frame_free(&frame);
            throw std::runtime_error("Could not allocate packet");
        }

        // Process audio in chunks
        size_t totalSamples = inputPcm.size();
        size_t frameSize = frame->nb_samples * mChannels;
        size_t position = 0;

        while (position < totalSamples) {
            size_t samplesToProcess = std::min(frameSize, totalSamples - position);
            if (samplesToProcess < frameSize) {
                // Last frame might be smaller
                av_frame_free(&frame);
                frame = av_frame_alloc();
                frame->format = AV_SAMPLE_FMT_S16P;

                // Set up channel layout for the frame
                av_channel_layout_from_string(&frame->ch_layout, "stereo");

                frame->sample_rate = mSampleRate;
                frame->nb_samples = samplesToProcess / mChannels;
                if (av_frame_get_buffer(frame, 0) < 0) {
                    av_frame_free(&frame);
                    av_packet_free(&pkt);
                    throw std::runtime_error("Could not allocate final audio frame");
                }
            }

            // Convert float PCM to signed 16-bit planar
            const uint8_t* inData[1] = { reinterpret_cast<const uint8_t*>(&inputPcm[position]) };
            swr_convert(mEncodeResampler, frame->data, frame->nb_samples,
                       inData, frame->nb_samples);

            // Encode frame
            int ret = avcodec_send_frame(mEncodeContext, frame);
            if (ret < 0) {
                av_frame_free(&frame);
                av_packet_free(&pkt);
                throw std::runtime_error("Error sending frame for encoding");
            }

            while (ret >= 0) {
                ret = avcodec_receive_packet(mEncodeContext, pkt);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    break;
                } else if (ret < 0) {
                    av_frame_free(&frame);
                    av_packet_free(&pkt);
                    throw std::runtime_error("Error during encoding");
                }

                // Store the encoded packet
                mp3Data.insert(mp3Data.end(), pkt->data, pkt->data + pkt->size);
                av_packet_unref(pkt);
            }

            position += samplesToProcess;
        }

        // Flush encoder
        avcodec_send_frame(mEncodeContext, nullptr);
        int ret = 0;
        while (ret >= 0) {
            ret = avcodec_receive_packet(mEncodeContext, pkt);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            } else if (ret < 0) {
                av_frame_free(&frame);
                av_packet_free(&pkt);
                throw std::runtime_error("Error during flushing encoder");
            }

            mp3Data.insert(mp3Data.end(), pkt->data, pkt->data + pkt->size);
            av_packet_unref(pkt);
        }

        av_frame_free(&frame);
        av_packet_free(&pkt);
        
        return mp3Data;
    }

    std::vector<float> DecodeMp3ToPcm(const std::vector<uint8_t>& mp3Data) {
        std::vector<float> outputPcm;
        
        // Create packet for input
        AVPacket* pkt = av_packet_alloc();
        if (!pkt) {
            throw std::runtime_error("Could not allocate packet");
        }

        // Create frame for output
        AVFrame* frame = av_frame_alloc();
        if (!frame) {
            av_packet_free(&pkt);
            throw std::runtime_error("Could not allocate frame");
        }

        // Process in chunks
        size_t position = 0;
        while (position < mp3Data.size()) {
            // Find a reasonable packet size (this is simplified)
            size_t packetSize = std::min<size_t>(1024, mp3Data.size() - position);
            
            av_packet_unref(pkt);
            pkt->data = const_cast<uint8_t*>(&mp3Data[position]);
            pkt->size = packetSize;
            
            // Send packet to decoder
            int ret = avcodec_send_packet(mDecodeContext, pkt);
            if (ret < 0) {
                av_frame_free(&frame);
                av_packet_free(&pkt);
                throw std::runtime_error("Error sending packet for decoding");
            }
            
            // Receive frames
            while (ret >= 0) {
                ret = avcodec_receive_frame(mDecodeContext, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    break;
                } else if (ret < 0) {
                    av_frame_free(&frame);
                    av_packet_free(&pkt);
                    throw std::runtime_error("Error during decoding");
                }
                
                // Allocate output buffer for converted samples
                size_t outSamples = frame->nb_samples * mChannels;
                size_t currentSize = outputPcm.size();
                outputPcm.resize(currentSize + outSamples);
                
                // Convert S16P to float
                uint8_t* outData[1] = { reinterpret_cast<uint8_t*>(&outputPcm[currentSize]) };
                swr_convert(mDecodeResampler, outData, frame->nb_samples,
                           const_cast<const uint8_t**>(frame->data), frame->nb_samples);
            }
            
            position += packetSize;
        }
        
        // Flush decoder
        avcodec_send_packet(mDecodeContext, nullptr);
        int ret = 0;
        while (ret >= 0) {
            ret = avcodec_receive_frame(mDecodeContext, frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            } else if (ret < 0) {
                av_frame_free(&frame);
                av_packet_free(&pkt);
                throw std::runtime_error("Error during flushing decoder");
            }
            
            // Allocate output buffer for converted samples
            size_t outSamples = frame->nb_samples * mChannels;
            size_t currentSize = outputPcm.size();
            outputPcm.resize(currentSize + outSamples);
            
            // Convert S16P to float
            uint8_t* outData[1] = { reinterpret_cast<uint8_t*>(&outputPcm[currentSize]) };
            swr_convert(mDecodeResampler, outData, frame->nb_samples,
                       const_cast<const uint8_t**>(frame->data), frame->nb_samples);
        }
        
        av_frame_free(&frame);
        av_packet_free(&pkt);
        
        return outputPcm;
    }
};