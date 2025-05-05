#pragma once

#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <choc_DisableAllWarnings.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}
#include <choc_ReenableAllWarnings.h>

// RAII wrappers for FFmpeg resources
namespace
{
    // RAII wrapper for AVFrame
    class ScopedAVFrame
    {
    public:
        ScopedAVFrame() : frame (av_frame_alloc())
        {
            if (!frame)
                throw std::runtime_error ("Could not allocate AVFrame");
        }

        ~ScopedAVFrame()
        {
            if (frame)
                av_frame_free (&frame);
        }

        AVFrame* get() const { return frame; }
        AVFrame* operator->() const { return frame; }

        // Disallow copying
        ScopedAVFrame (const ScopedAVFrame&) = delete;
        ScopedAVFrame& operator= (const ScopedAVFrame&) = delete;

    private:
        AVFrame* frame;
    };

    // RAII wrapper for AVPacket
    class ScopedAVPacket
    {
    public:
        ScopedAVPacket() : pkt (av_packet_alloc())
        {
            if (!pkt)
                throw std::runtime_error ("Could not allocate AVPacket");
        }

        ~ScopedAVPacket()
        {
            if (pkt)
                av_packet_free (&pkt);
        }

        AVPacket* get() const { return pkt; }
        AVPacket* operator->() const { return pkt; }

        // Disallow copying
        ScopedAVPacket (const ScopedAVPacket&) = delete;
        ScopedAVPacket& operator= (const ScopedAVPacket&) = delete;

    private:
        AVPacket* pkt;
    };

    // RAII wrapper for AVChannelLayout
    class ScopedAVChannelLayout
    {
    public:
        ScopedAVChannelLayout()
        {
            av_channel_layout_default (&layout, 2); // Default to stereo
        }

        explicit ScopedAVChannelLayout (const char* layoutStr)
        {
            av_channel_layout_from_string (&layout, layoutStr);
        }

        ~ScopedAVChannelLayout()
        {
            av_channel_layout_uninit (&layout);
        }

        AVChannelLayout* get() { return &layout; }
        const AVChannelLayout* get() const { return &layout; }

        // Disallow copying
        ScopedAVChannelLayout (const ScopedAVChannelLayout&) = delete;
        ScopedAVChannelLayout& operator= (const ScopedAVChannelLayout&) = delete;

    private:
        AVChannelLayout layout = { .order = AV_CHANNEL_ORDER_UNSPEC, .nb_channels = 0 };
    };
}

class MP3Effect
{
public:
    // Configure the MP3 effect with sample rate, channels, and bitrate (in kbps)
    MP3Effect (int sampleRate, int channels, int bitrate = 128)
        : mSampleRate (sampleRate), mChannels (channels), mBitrate (bitrate), mInputBuffer (32768) // Pre-allocate working buffers
          ,
          mOutputBuffer (32768),
          mMp3Buffer (32768) // Pre-allocate MP3 buffer with initial size
    {
        Initialize();
    }

    ~MP3Effect()
    {
        Cleanup();
    }

    // Process PCM audio through MP3 encoding/decoding using raw pointers
    // buffers[0] contains input samples for left channel, buffers[1] for right channel (if stereo)
    // After processing, output samples replace the input in the same buffers
    void Process (float* const* buffers, int numSamples)
    {
        if (!mIsInitialized)
        {
            throw std::runtime_error ("MP3Effect not properly initialized");
        }

        // Ensure our working buffers are large enough
        EnsureBufferSize (mInputBuffer, numSamples * mChannels);
        EnsureBufferSize (mOutputBuffer, numSamples * mChannels);
        // Ensure MP3 buffer is large enough (worst case MP3 size is 1.25x PCM)
        EnsureBufferSize (mMp3Buffer, static_cast<int> (numSamples * mChannels * 1.25));

        // Interleave input channels into our working buffer
        if (mChannels == 1)
        {
            std::memcpy (mInputBuffer.data(), buffers[0], numSamples * sizeof (float));
        }
        else
        {
            // Interleave stereo channels
            for (int i = 0; i < numSamples; ++i)
            {
                for (int ch = 0; ch < mChannels; ++ch)
                {
                    mInputBuffer[i * mChannels + ch] = buffers[ch][i];
                }
            }
        }

        // Process through MP3 codec
        ProcessInternal (mInputBuffer.data(), numSamples * mChannels, mOutputBuffer.data());

        // De-interleave output back to separate channel buffers
        if (mChannels == 1)
        {
            std::memcpy (buffers[0], mOutputBuffer.data(), numSamples * sizeof (float));
        }
        else
        {
            // De-interleave stereo channels
            for (int i = 0; i < numSamples; ++i)
            {
                for (int ch = 0; ch < mChannels; ++ch)
                {
                    buffers[ch][i] = mOutputBuffer[i * mChannels + ch];
                }
            }
        }
    }

    // Change the MP3 bitrate (in kbps)
    void SetBitrate (int bitrate)
    {
        if (mBitrate == bitrate)
            return;

        mBitrate = bitrate;
        Cleanup();
        Initialize();
    }

private:
    // Audio configuration
    int mSampleRate;
    int mChannels;
    int mBitrate;

    // Pre-allocated working buffers
    std::vector<float> mInputBuffer;
    std::vector<float> mOutputBuffer;
    std::vector<uint8_t> mMp3Buffer;

    // Helper method to ensure buffer size
    template <typename T>
    void EnsureBufferSize (std::vector<T>& buffer, int requiredSize)
    {
        if (buffer.size() < static_cast<size_t> (requiredSize))
        {
            buffer.resize (requiredSize);
        }
    }

    // Reusable frames and packets for encoding/decoding
    ScopedAVFrame mEncodeFrame;
    ScopedAVFrame mDecodeFrame;
    ScopedAVPacket mEncodePacket;
    ScopedAVPacket mDecodePacket;

    // FFmpeg context objects
    AVCodecContext* mEncodeContext = nullptr;
    AVCodecContext* mDecodeContext = nullptr;
    SwrContext* mEncodeResampler = nullptr;
    SwrContext* mDecodeResampler = nullptr;

    bool mIsInitialized = false;
    static constexpr int kMp3FrameSize = 1152; // Standard MP3 frame size

    void Initialize()
    {
        // Set up encoder
        const AVCodec* encoder = avcodec_find_encoder (AV_CODEC_ID_MP3);
        if (!encoder)
        {
            throw std::runtime_error ("Could not find MP3 encoder");
        }

        mEncodeContext = avcodec_alloc_context3 (encoder);
        if (!mEncodeContext)
        {
            throw std::runtime_error ("Could not allocate encoder context");
        }

        mEncodeContext->bit_rate = mBitrate * 1'000;
        mEncodeContext->sample_rate = mSampleRate;

        // Set channel layout using the new API
        av_channel_layout_default (&mEncodeContext->ch_layout, mChannels);
        if (mChannels == 2)
        {
            av_channel_layout_from_string (&mEncodeContext->ch_layout, "stereo");
        }

        mEncodeContext->sample_fmt = AV_SAMPLE_FMT_S16P; // MP3 encoder uses planar signed 16-bit

        if (avcodec_open2 (mEncodeContext, encoder, nullptr) < 0)
        {
            Cleanup();
            throw std::runtime_error ("Could not open encoder");
        }

        // Set up decoder
        const AVCodec* decoder = avcodec_find_decoder (AV_CODEC_ID_MP3);
        if (!decoder)
        {
            Cleanup();
            throw std::runtime_error ("Could not find MP3 decoder");
        }

        mDecodeContext = avcodec_alloc_context3 (decoder);
        if (!mDecodeContext)
        {
            Cleanup();
            throw std::runtime_error ("Could not allocate decoder context");
        }

        mDecodeContext->bit_rate = mBitrate * 1'000;
        mDecodeContext->sample_rate = mSampleRate;

        // Set channel layout using the new API
        av_channel_layout_default (&mDecodeContext->ch_layout, mChannels);
        if (mChannels == 2)
        {
            av_channel_layout_from_string (&mDecodeContext->ch_layout, "stereo");
        }

        mDecodeContext->sample_fmt = AV_SAMPLE_FMT_S16P;

        if (avcodec_open2 (mDecodeContext, decoder, nullptr) < 0)
        {
            Cleanup();
            throw std::runtime_error ("Could not open decoder");
        }

        // Set up resamplers for format conversion
        ScopedAVChannelLayout stereoLayout ("stereo");

        // Setup encode resampler
        mEncodeResampler = swr_alloc();
        if (!mEncodeResampler)
        {
            Cleanup();
            throw std::runtime_error ("Could not allocate encode resampler");
        }

        // Set channel layouts using the new API
        av_opt_set_chlayout (mEncodeResampler, "in_chlayout", stereoLayout.get(), 0);
        av_opt_set_chlayout (mEncodeResampler, "out_chlayout", stereoLayout.get(), 0);

        av_opt_set_int (mEncodeResampler, "in_sample_rate", mSampleRate, 0);
        av_opt_set_int (mEncodeResampler, "out_sample_rate", mSampleRate, 0);
        av_opt_set_sample_fmt (mEncodeResampler, "in_sample_fmt", AV_SAMPLE_FMT_FLT, 0);
        av_opt_set_sample_fmt (mEncodeResampler, "out_sample_fmt", AV_SAMPLE_FMT_S16P, 0);

        if (swr_init (mEncodeResampler) < 0)
        {
            Cleanup();
            throw std::runtime_error ("Could not initialize encode resampler");
        }

        // Setup decode resampler
        mDecodeResampler = swr_alloc();
        if (!mDecodeResampler)
        {
            Cleanup();
            throw std::runtime_error ("Could not allocate decode resampler");
        }

        // Set channel layouts using the new API
        av_opt_set_chlayout (mDecodeResampler, "in_chlayout", stereoLayout.get(), 0);
        av_opt_set_chlayout (mDecodeResampler, "out_chlayout", stereoLayout.get(), 0);

        av_opt_set_int (mDecodeResampler, "in_sample_rate", mSampleRate, 0);
        av_opt_set_int (mDecodeResampler, "out_sample_rate", mSampleRate, 0);
        av_opt_set_sample_fmt (mDecodeResampler, "in_sample_fmt", AV_SAMPLE_FMT_S16P, 0);
        av_opt_set_sample_fmt (mDecodeResampler, "out_sample_fmt", AV_SAMPLE_FMT_FLT, 0);

        if (swr_init (mDecodeResampler) < 0)
        {
            Cleanup();
            throw std::runtime_error ("Could not initialize decode resampler");
        }

        // Pre-configure the encode frame to avoid allocations during processing
        mEncodeFrame->format = AV_SAMPLE_FMT_S16P;
        av_channel_layout_from_string (&mEncodeFrame->ch_layout, "stereo");
        mEncodeFrame->sample_rate = mSampleRate;
        mEncodeFrame->nb_samples = kMp3FrameSize;

        if (av_frame_get_buffer (mEncodeFrame.get(), 0) < 0)
        {
            Cleanup();
            throw std::runtime_error ("Could not allocate audio frame buffer");
        }

        mIsInitialized = true;
    }

    void Cleanup()
    {
        if (mEncodeContext)
        {
            avcodec_free_context (&mEncodeContext);
            mEncodeContext = nullptr;
        }

        if (mDecodeContext)
        {
            avcodec_free_context (&mDecodeContext);
            mDecodeContext = nullptr;
        }

        if (mEncodeResampler)
        {
            swr_free (&mEncodeResampler);
            mEncodeResampler = nullptr;
        }

        if (mDecodeResampler)
        {
            swr_free (&mDecodeResampler);
            mDecodeResampler = nullptr;
        }

        mIsInitialized = false;
    }

    // Process audio internally - processes interleaved audio through MP3 codec
    void ProcessInternal (const float* inputBuffer, int numSamples, float* outputBuffer)
    {
        // Encode PCM to MP3
        EncodePcmToMp3 (inputBuffer, numSamples);

        // Decode MP3 back to PCM
        DecodeMp3ToPcm (outputBuffer);
    }

    void EncodePcmToMp3 (const float* inputBuffer, int numSamples)
    {
        mMp3Buffer.clear();

        // Process audio in chunks
        int frameSize = kMp3FrameSize * mChannels;
        int position = 0;

        while (position < numSamples)
        {
            int samplesToProcess = std::min (frameSize, numSamples - position);

            // Handle last, partial frame if needed
            if (samplesToProcess < frameSize)
            {
                // Create a temporary frame with the right size
                ScopedAVFrame lastFrame;
                lastFrame->format = AV_SAMPLE_FMT_S16P;
                av_channel_layout_from_string (&lastFrame->ch_layout, "stereo");
                lastFrame->sample_rate = mSampleRate;
                lastFrame->nb_samples = samplesToProcess / mChannels;

                if (av_frame_get_buffer (lastFrame.get(), 0) < 0)
                {
                    throw std::runtime_error ("Could not allocate final audio frame");
                }

                // Convert float PCM to signed 16-bit planar
                const uint8_t* inData[1] = { reinterpret_cast<const uint8_t*> (&inputBuffer[position]) };
                swr_convert (mEncodeResampler, lastFrame->data, lastFrame->nb_samples, inData, lastFrame->nb_samples);

                // Encode this last frame
                EncodeFrame (lastFrame.get(), mMp3Buffer);
            }
            else
            {
                // Standard full frame processing
                // Convert float PCM to signed 16-bit planar
                const uint8_t* inData[1] = { reinterpret_cast<const uint8_t*> (&inputBuffer[position]) };
                swr_convert (mEncodeResampler, mEncodeFrame->data, mEncodeFrame->nb_samples, inData, mEncodeFrame->nb_samples);

                // Encode frame
                EncodeFrame (mEncodeFrame.get(), mMp3Buffer);
            }

            position += samplesToProcess;
        }

        // Flush encoder
        EncodeFrame (nullptr, mMp3Buffer);
    }

    // Helper to encode a single frame and append to mp3Data
    void EncodeFrame (AVFrame* frame, std::vector<uint8_t>& mp3Data)
    {
        int ret = avcodec_send_frame (mEncodeContext, frame);
        if (ret < 0 && ret != AVERROR_EOF)
        {
            throw std::runtime_error ("Error sending frame for encoding");
        }

        while (true)
        {
            ret = avcodec_receive_packet (mEncodeContext, mEncodePacket.get());
            if (ret == AVERROR (EAGAIN) || ret == AVERROR_EOF)
            {
                break;
            }
            else if (ret < 0)
            {
                throw std::runtime_error ("Error during encoding");
            }

            // Store the encoded packet
            mp3Data.insert (mp3Data.end(), mEncodePacket->data, mEncodePacket->data + mEncodePacket->size);
            av_packet_unref (mEncodePacket.get());
        }
    }

    // Decode MP3 data to PCM buffer
    void DecodeMp3ToPcm (float* outputBuffer)
    {
        int totalSamplesWritten = 0;

        // Process in chunks
        size_t position = 0;
        while (position < mMp3Buffer.size())
        {
            // Find a reasonable packet size
            size_t packetSize = std::min<size_t> (1024, mMp3Buffer.size() - position);

            av_packet_unref (mDecodePacket.get());
            mDecodePacket->data = const_cast<uint8_t*> (&mMp3Buffer[position]);
            mDecodePacket->size = static_cast<int> (packetSize);

            // Send packet to decoder
            int ret = avcodec_send_packet (mDecodeContext, mDecodePacket.get());
            if (ret < 0 && ret != AVERROR_EOF)
            {
                throw std::runtime_error ("Error sending packet for decoding");
            }

            // Receive frames
            while (true)
            {
                ret = avcodec_receive_frame (mDecodeContext, mDecodeFrame.get());
                if (ret == AVERROR (EAGAIN) || ret == AVERROR_EOF)
                {
                    break;
                }
                else if (ret < 0)
                {
                    throw std::runtime_error ("Error during decoding");
                }

                // Convert from signed 16-bit planar to float
                uint8_t* outData[1] = { reinterpret_cast<uint8_t*> (&outputBuffer[totalSamplesWritten]) };
                int samplesConverted = swr_convert (mDecodeResampler, outData, mDecodeFrame->nb_samples, const_cast<const uint8_t**> (mDecodeFrame->data), mDecodeFrame->nb_samples);

                totalSamplesWritten += samplesConverted * mChannels;
                av_frame_unref (mDecodeFrame.get());
            }

            position += packetSize;
        }

        // Flush the resampler
        uint8_t* outData[1] = { reinterpret_cast<uint8_t*> (&outputBuffer[totalSamplesWritten]) };
        int samplesConverted = swr_convert (mDecodeResampler, outData, 1024, nullptr, 0);
        if (samplesConverted > 0)
        {
            totalSamplesWritten += samplesConverted * mChannels;
        }
    }
};