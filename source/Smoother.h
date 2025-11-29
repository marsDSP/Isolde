#pragma once

#include <Includes.h>

namespace MarsDSP
{
    template <typename ParametersType>
    class Smoother
    {
    public:
        explicit Smoother(const ParametersType& p) : params(p) {}

        void prepare(const juce::dsp::ProcessSpec& spec) noexcept
        {
            constexpr double duration = 0.02;
            const int steps = static_cast<int>(spec.sampleRate * duration);
            auto resetAll = [steps](auto& smootherArray)
            {
                for (auto& smoother : smootherArray)
                    smoother.reset(steps);
            };

            resetAll(speedSmoother);
            resetAll(randoSmoother);
            resetAll(depthSmoother);
            resetAll(regenSmoother);
            resetAll(derezSmoother);
            resetAll(bufferSmoother);
            resetAll(outputSmoother);
            resetAll(drywetSmoother);
        }

        void reset() noexcept
        {
            speed = 0.0f;
            for (auto& smoother : speedSmoother)
                smoother.setCurrentAndTargetValue(params.speed->get());

            rando = 0.0f;
            for (auto& smoother : randoSmoother)
                smoother.setCurrentAndTargetValue(params.rando->get());

            depth = 1.0f;
            for (auto& smoother : depthSmoother)
                smoother.setCurrentAndTargetValue(params.depth->get());

            regen = 0.0f;
            for (auto& smoother : regenSmoother)
                smoother.setCurrentAndTargetValue(params.regen->get());

            derez = 0.0f;
            for (auto& smoother : derezSmoother)
                smoother.setCurrentAndTargetValue(params.derez->get());

            buffer = 0.0f;
            for (auto& smoother : bufferSmoother)
                smoother.setCurrentAndTargetValue(params.buffer->get());

            output = 0.0f;
            for (auto& smoother : outputSmoother)
                smoother.setCurrentAndTargetValue(params.output->get());

            drywet = 0.0f;
            for (auto& smoother : drywetSmoother)
                smoother.setCurrentAndTargetValue(params.drywet->get());
        }

        void update() noexcept
        {
            const float speedNew = params.speed->get();
            for (auto& smoother : speedSmoother)
                smoother.setTargetValue(speedNew);

            const float randoNew = params.rando->get();
            for (auto& smoother : speedSmoother)
                smoother.setTargetValue(randoNew);

            const float depthNew = params.depth->get();
            for (auto& smoother : randoSmoother)
                smoother.setTargetValue(depthNew);

            const float regenNew = params.regen->get();
            for (auto& smoother : depthSmoother)
                smoother.setTargetValue(regenNew);

            const float derezNew = params.derez->get();
            for (auto& smoother : regenSmoother)
                smoother.setTargetValue(derezNew);

            const float bufferNew = params.buffer->get();
            for (auto& smoother : derezSmoother)
                smoother.setTargetValue(bufferNew);

            const float outDB = params.output->get();
            for (auto& smoother : outputSmoother)
                smoother.setTargetValue(outDB);

            const float dryWetNew = params.drywet->get();
            for (auto& smoother : drywetSmoother)
                smoother.setTargetValue(dryWetNew);

            isBypassed = params.bypass->get();
        }

        void smoothen() noexcept
        {
            auto smoothen = [](auto& smootherArray)
            {
                for (auto& smoother : smootherArray)
                    smoother.getNextValue();
            };

            smoothen(speedSmoother);
            smoothen(randoSmoother);
            smoothen(depthSmoother);
            smoothen(regenSmoother);
            smoothen(derezSmoother);
            smoothen(bufferSmoother);
            smoothen(outputSmoother);
            smoothen(drywetSmoother);
        }


        enum class SmootherUpdateMode
        {
            initialize,
            liveInRealTime
        };

        void setSmoother(int numSamplesToSkip, SmootherUpdateMode init) noexcept
        {
            juce::ignoreUnused(init);

            auto skipArray = [numSamplesToSkip](auto& smootherArray)
            {
                for (auto& s : smootherArray)
                    s.skip(numSamplesToSkip);
            };

            skipArray(speedSmoother);
            skipArray(randoSmoother);
            skipArray(depthSmoother);
            skipArray(regenSmoother);
            skipArray(derezSmoother);
            skipArray(bufferSmoother);
            skipArray(outputSmoother);
            skipArray(drywetSmoother);
        }

        // A
        float getSpeed(size_t channel = 0) noexcept
        { return speedSmoother[channel].getNextValue(); }

        // B
        float getRando(size_t channel = 0) noexcept
        { return randoSmoother[channel].getNextValue(); }

        // C
        float getDepth(size_t channel = 0) noexcept
        { return depthSmoother[channel].getNextValue(); }

        // D
        float getRegen(size_t channel = 0) noexcept
        { return regenSmoother[channel].getNextValue(); }

        // E
        float getDerez(size_t channel = 0) noexcept
        { return derezSmoother[channel].getNextValue(); }

        // F
        float getBuffer(size_t channel = 0) noexcept
        { return bufferSmoother[channel].getNextValue(); }

        // G
        float getOutput(size_t channel = 0) noexcept
        { return outputSmoother[channel].getNextValue(); }

        // H
        float getDryWet(size_t channel = 0) noexcept
        { return drywetSmoother[channel].getNextValue(); }

        // I
        [[nodiscard]] bool getBypass() const noexcept
        { return isBypassed; }

    private:

        const ParametersType& params;

        float speed     { 0.0f };
        float rando     { 0.0f };
        float depth     { 0.0f };
        float regen     { 0.0f };
        float derez     { 0.0f };
        float buffer    { 0.0f };
        float output    { 0.0f };
        float drywet    { 0.0f };

        bool isBypassed { false };

        std::array<juce::LinearSmoothedValue<float>, 2>

        speedSmoother,
        randoSmoother,
        depthSmoother,
        regenSmoother,
        derezSmoother,
        bufferSmoother,
        outputSmoother,
        drywetSmoother;
    };
}
