#pragma once

#include <Includes.h>
#include "Globals.h"

namespace MarsDSP {

    class Parameters {
    public:

        explicit Parameters(juce::AudioProcessorValueTreeState &vts)
        {
            castParameter(vts, speedParamID,  speed);
            castParameter(vts, randoParamID,  rando);
            castParameter(vts, depthParamID,  depth);
            castParameter(vts, regenParamID,  regen);
            castParameter(vts, derezParamID,  derez);
            castParameter(vts, bufferParamID, buffer);
            castParameter(vts, outputParamID, output);
            castParameter(vts, drywetParamID, drywet);
            castParameter(vts, bypassParamID, bypass);
        }

        static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout()
        {
            juce::AudioProcessorValueTreeState::ParameterLayout layout;

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (speedParamID, speedParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.5f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (randoParamID, randoParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.75f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (depthParamID, depthParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.75f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (regenParamID, regenParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.1f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (derezParamID, derezParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.5f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (bufferParamID, bufferParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.05f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (outputParamID, outputParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.5f));

            layout.add(std::make_unique<juce::AudioParameterFloat>
                (drywetParamID, drywetParamIDName, juce::NormalisableRange<float>
                { 0.0f, 1.0f }, 0.75f));

            // Bypass
            layout.add(std::make_unique<juce::AudioParameterBool>
                (bypassParamID, bypassParamIDName, false));

            return layout;
        }

        ~Parameters() = default;

        juce::AudioParameterFloat* speed  {nullptr};
        juce::AudioParameterFloat* rando  {nullptr};
        juce::AudioParameterFloat* depth  {nullptr};
        juce::AudioParameterFloat* regen  {nullptr};
        juce::AudioParameterFloat* derez  {nullptr};
        juce::AudioParameterFloat* buffer {nullptr};
        juce::AudioParameterFloat* output {nullptr};
        juce::AudioParameterFloat* drywet {nullptr};

        juce::AudioParameterBool*  bypass {nullptr};

    private:

        template<typename T>
        static void castParameter(juce::AudioProcessorValueTreeState& vts, const juce::ParameterID& id, T& destination)
        {
            destination = dynamic_cast<T>(vts.getParameter(id.getParamID()));
            jassert(destination);
        }

        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(Parameters)
    };
}
