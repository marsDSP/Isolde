#pragma once

#include <Includes.h>
#include "Parameters.h"
#include "Smoother.h"
#include "ModDSP.h"

namespace MarsDSP::DSP {

    class ProcessBlock {
    public:

        ProcessBlock() = default;
        ~ProcessBlock() = default;

        void prepareDSP (double sampleRate, juce::uint32 samplesPerBlock,
                        juce::uint32 numChannels, const Parameters& params)
        {
            spec.sampleRate = sampleRate;
            spec.maximumBlockSize = samplesPerBlock;
            spec.numChannels = numChannels;

            smoother = std::make_unique<Smoother<Parameters>>(params);
            smoother->prepare(spec);
            smoother->reset();

            mod.prepare(spec);

            scratchBuffer.setSize(1, (int) samplesPerBlock);
        }

        void process (juce::AudioBuffer<float>& buffer)
        {
            if (smoother)
            {
                smoother->update();
                if (smoother->getBypass())
                    return;
            }

            const auto numChannels = buffer.getNumChannels();
            const auto numSamples = buffer.getNumSamples();

            auto* inL = buffer.getReadPointer(0);
            auto* inR = (numChannels > 1) ? buffer.getReadPointer(1) : inL;

            auto* outL = buffer.getWritePointer(0);
            auto* outR = (numChannels > 1) ? buffer.getWritePointer(1) : scratchBuffer.getWritePointer(0);

            mod.processMod(inL, inR, outL, outR, numSamples, *smoother);
        }

    private:

        juce::dsp::ProcessSpec spec {};
        std::unique_ptr<juce::dsp::Oversampling<float>> m_oversample;
        std::unique_ptr<Smoother<Parameters>> smoother;
        ModDSP mod;
        juce::AudioBuffer<float> scratchBuffer;
    };
}
