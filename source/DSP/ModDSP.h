#pragma once

#include <Includes.h>
#include <array>
#include <cmath>

namespace MarsDSP::DSP {

    class ModDSP {

    public:

    	ModDSP() = default;

    	void reset ()
    	{

    		A = 0.15;
    		B = 0.5;
    		C = 0.5;
    		D = 0.5;
    		E = 0.5;
    		F = 0.5;
    		G = 1.0;
    		H = 1.0;

    		for(int x = 0; x < 32767+2; x++) {a3AL[x] = 0.0; a3AR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3BL[x] = 0.0; a3BR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3CL[x] = 0.0; a3CR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3DL[x] = 0.0; a3DR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3EL[x] = 0.0; a3ER[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3FL[x] = 0.0; a3FR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3GL[x] = 0.0; a3GR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3HL[x] = 0.0; a3HR[x] = 0.0;}
    		for(int x = 0; x < 32767+2; x++) {a3IL[x] = 0.0; a3IR[x] = 0.0;}

    		c3AL = c3BL = c3CL = c3DL = c3EL = c3FL = c3GL = c3HL = c3IL = 1;
    		c3AR = c3BR = c3CR = c3DR = c3ER = c3FR = c3GR = c3HR = c3IR = 1;
    		f3AL = f3BL = f3CL = 0.0;
    		f3CR = f3FR = f3IR = 0.0;
    		avg3L = avg3R = 0.0;

    		for (int x = 0; x < bez_total; x++) bez[x] = 0.0;
    		bez[bez_cycle] = 1.0;

    		rotate = 0.0;
    		oldfpd = 0.4294967295;

    		buf = 8192;
    		vibDepth = 0.0;
    		outA = 1.0;
    		outB = 1.0;
    		wetA = 1.0;
    		wetB = 1.0;
    		derezA = 1.0;
    		derezB = 1.0;

    		lastSampleL = 0.0;
    		wasPosClipL = false;
    		wasNegClipL = false;
    		lastSampleR = 0.0;
    		wasPosClipR = false;
    		wasNegClipR = false;

    		fpdL = 1.0; while (fpdL < 16386) fpdL = rand()*UINT32_MAX;
    		fpdR = 1.0; while (fpdR < 16386) fpdR = rand()*UINT32_MAX;
    	}

    	void prepare(juce::dsp::ProcessSpec& spec)
    	{
    		sampleRate = spec.sampleRate;
    		reset();
    	}

    	template <typename SmootherType>
    	void processMod(const float* inL, const float* inR,
    						float* outL, float* outR, int numSamples, SmootherType& smoother)
		{

    		if (inL == nullptr || inR == nullptr || outL == nullptr || outR == nullptr || numSamples <= 0)
    			return;

    		updateSmoother(smoother, numSamples);

    		const float* in1 = inL;
    		const float* in2 = inR;
    		float* out1 = outL;
    		float* out2 = outR;

			juce::uint32 inFramesToProcess = numSamples; //vst doesn't give us this as a separate variable so we'll make it
			double overallscale = 1.0;
			overallscale /= 44100.0;
			overallscale *= getSampleRate();
			int spacing = floor(overallscale); //should give us working basic scaling, usually 2 or 4
			if (spacing < 1) spacing = 1; if (spacing > 16) spacing = 16;
			double vibSpeed = pow(A,5.0) * 0.1;
			double vibRandom = pow(B,3.0) * 0.99;
			double targetDepth = pow(C,2.0) * 0.5;
			double reg3n = D*0.0625;

			derezA = derezB; derezB = E*2.0;
			bool stepped = true; // Revised Bezier Undersampling
			if (derezB > 1.0) {  // has full rez at center, stepped
				stepped = false; // to left, continuous to right
				derezB = 1.0-(derezB-1.0);
			} //if it's set up like that it's the revised algorithm
			derezB = fmin(fmax(derezB/overallscale,0.0025),1.0);
			int bezFraction = (int)(1.0/derezB);
			double bezTrim = (double)bezFraction/(bezFraction+1.0);
			if (stepped) { //this hard-locks derez to exact subdivisions of 1.0
				derezB = 1.0 / bezFraction;
				bezTrim = 1.0-(derezB*bezTrim);
			} else { //this makes it match the 1.0 case using stepped
				bezTrim = 1.0 / (1.0 + derezB);
			} //the revision more accurately connects the bezier curves

			int targetBuf = (pow(F,3.0)*32510.0)+256;
			outA = outB; outB = G;
			wetA = wetB; wetB = 1.0-pow(1.0-H,2.0);

	    	while (--numSamples >= 0)
	    	{
				double inputSampleL = *in1;
				double inputSampleR = *in2;
				if (fabs(inputSampleL)<1.18e-23) inputSampleL = fpdL * 1.18e-17;
				if (fabs(inputSampleR)<1.18e-23) inputSampleR = fpdR * 1.18e-17;
				double drySampleL = inputSampleL;
				double drySampleR = inputSampleR;
				double temp = (double)numSamples/inFramesToProcess;
				double derez = (derezA*temp)+(derezB*(1.0-temp));
				double out = (outA*temp)+(outB*(1.0-temp));
				double wet = (wetA*temp)+(wetB*(1.0-temp));

				bez[bez_cycle] += derez;
				bez[bez_SampL] += ((inputSampleL+bez[bez_InL]) * derez);
				bez[bez_SampR] += ((inputSampleR+bez[bez_InR]) * derez);
				bez[bez_InL] = inputSampleL; bez[bez_InR] = inputSampleR;
				if (bez[bez_cycle] > 1.0)
				{
					if (stepped) bez[bez_cycle] = 0.0;
					else bez[bez_cycle] -= 1.0;

					inputSampleL = (bez[bez_SampL]+bez[bez_AvgInSampL])*0.5;
					bez[bez_AvgInSampL] = bez[bez_SampL];
					inputSampleR = (bez[bez_SampR]+bez[bez_AvgInSampR])*0.5;
					bez[bez_AvgInSampR] = bez[bez_SampR];

					rotate += (oldfpd*vibSpeed);
					if (rotate > (M_PI*2.0)) {
						rotate -= (M_PI*2.0);
						oldfpd = (1.0-vibRandom) + ((fpdL*0.000000000618)*vibRandom);
					}
					vibDepth = (vibDepth*0.99)+(targetDepth*0.01);
					if (buf < targetBuf) buf++;
					if (buf > targetBuf) buf--;

					double mA = (sin(rotate)+1.0)*vibDepth*buf;
					double mB = (sin(rotate+(M_PI/9.0))+1.0)*vibDepth*buf;
					double mC = (sin(rotate+((M_PI/9.0)*2.0))+1.0)*vibDepth*buf;
					double mD = (sin(rotate+((M_PI/9.0)*3.0))+1.0)*vibDepth*buf;
					double mE = (sin(rotate+((M_PI/9.0)*4.0))+1.0)*vibDepth*buf;
					double mF = (sin(rotate+((M_PI/9.0)*5.0))+1.0)*vibDepth*buf;
					double mG = (sin(rotate+((M_PI/9.0)*6.0))+1.0)*vibDepth*buf;
					double mH = (sin(rotate+((M_PI/9.0)*7.0))+1.0)*vibDepth*buf;
					double mI = (sin(rotate+((M_PI/9.0)*8.0))+1.0)*vibDepth*buf;

					inputSampleL = sin(fmin(fmax(inputSampleL*0.5,-M_PI_2),M_PI_2));
					inputSampleR = sin(fmin(fmax(inputSampleR*0.5,-M_PI_2),M_PI_2));

					a3AL[c3AL] = inputSampleL + fmin(fmax(f3AL*reg3n,-M_PI),M_PI);
					a3BL[c3BL] = inputSampleL + fmin(fmax(f3BL*reg3n,-M_PI),M_PI);
					a3CL[c3CL] = inputSampleL + fmin(fmax(f3CL*reg3n,-M_PI),M_PI);

					a3CR[c3CR] = inputSampleR + fmin(fmax(f3CR*reg3n,-M_PI),M_PI);
					a3FR[c3FR] = inputSampleR + fmin(fmax(f3FR*reg3n,-M_PI),M_PI);
					a3IR[c3IR] = inputSampleR + fmin(fmax(f3IR*reg3n,-M_PI),M_PI);

					c3AL++; if (c3AL < 0 || c3AL > buf) c3AL = 0;
					c3BL++; if (c3BL < 0 || c3BL > buf) c3BL = 0;
					c3CL++; if (c3CL < 0 || c3CL > buf) c3CL = 0;
					c3CR++; if (c3CR < 0 || c3CR > buf) c3CR = 0;
					c3FR++; if (c3FR < 0 || c3FR > buf) c3FR = 0;
					c3IR++; if (c3IR < 0 || c3IR > buf) c3IR = 0;

					double o3AL = a3AL[(int)(c3AL+mA-((c3AL+mA>buf)?buf+1:0))] *(1.0-(mA-floor(mA)));
					o3AL += (a3AL[(int)(c3AL+mA+1-((c3AL+mA+1>buf)?buf+1:0))] *((mA-floor(mA))));
					double o3BL = a3BL[(int)(c3BL+mB-((c3BL+mB>buf)?buf+1:0))] *(1.0-(mB-floor(mB)));
					o3BL += (a3BL[(int)(c3BL+mB+1-((c3BL+mB+1>buf)?buf+1:0))] *((mB-floor(mB))));
					double o3CL = a3CL[(int)(c3CL+mC-((c3CL+mC>buf)?buf+1:0))] *(1.0-(mC-floor(mC)));
					o3CL += (a3CL[(int)(c3CL+mC+1-((c3CL+mC+1>buf)?buf+1:0))] *((mC-floor(mC))));
					double o3CR = a3CR[(int)(c3CR+mC-((c3CR+mC>buf)?buf+1:0))] *(1.0-(mC-floor(mC)));
					o3CR += (a3CR[(int)(c3CR+mC+1-((c3CR+mC+1>buf)?buf+1:0))] *((mC-floor(mC))));
					double o3FR = a3FR[(int)(c3FR+mF-((c3FR+mF>buf)?buf+1:0))] *(1.0-(mF-floor(mF)));
					o3FR += (a3FR[(int)(c3FR+mF+1-((c3FR+mF+1>buf)?buf+1:0))] *((mF-floor(mF))));
					double o3IR = a3IR[(int)(c3IR+mI-((c3IR+mI>buf)?buf+1:0))] *(1.0-(mI-floor(mI)));
					o3IR += (a3IR[(int)(c3IR+mI+1-((c3IR+mI+1>buf)?buf+1:0))] *((mI-floor(mI))));

					a3DL[c3DL] = (((o3BL + o3CL) * -2.0) + o3AL);
					a3EL[c3EL] = (((o3AL + o3CL) * -2.0) + o3BL);
					a3FL[c3FL] = (((o3AL + o3BL) * -2.0) + o3CL);
					a3BR[c3BR] = (((o3FR + o3IR) * -2.0) + o3CR);
					a3ER[c3ER] = (((o3CR + o3IR) * -2.0) + o3FR);
					a3HR[c3HR] = (((o3CR + o3FR) * -2.0) + o3IR);

					c3DL++; if (c3DL < 0 || c3DL > buf) c3DL = 0;
					c3EL++; if (c3EL < 0 || c3EL > buf) c3EL = 0;
					c3FL++; if (c3FL < 0 || c3FL > buf) c3FL = 0;
					c3BR++; if (c3BR < 0 || c3BR > buf) c3BR = 0;
					c3ER++; if (c3ER < 0 || c3ER > buf) c3ER = 0;
					c3HR++; if (c3HR < 0 || c3HR > buf) c3HR = 0;

					double o3DL = a3DL[(int)(c3DL+mD-((c3DL+mD>buf)?buf+1:0))] *(1.0-(mD-floor(mD)));
					o3DL += (a3DL[(int)(c3DL+mD+1-((c3DL+mD+1>buf)?buf+1:0))] *((mD-floor(mD))));
					double o3EL = a3EL[(int)(c3EL+mE-((c3EL+mE>buf)?buf+1:0))] *(1.0-(mE-floor(mE)));
					o3EL += (a3EL[(int)(c3EL+mE+1-((c3EL+mE+1>buf)?buf+1:0))] *((mE-floor(mE))));
					double o3FL = a3FL[(int)(c3FL+mF-((c3FL+mF>buf)?buf+1:0))] *(1.0-(mF-floor(mF)));
					o3FL += (a3FL[(int)(c3FL+mF+1-((c3FL+mF+1>buf)?buf+1:0))] *((mF-floor(mF))));
					double o3BR = a3BR[(int)(c3BR+mB-((c3BR+mB>buf)?buf+1:0))] *(1.0-(mB-floor(mB)));
					o3BR += (a3BR[(int)(c3BR+mB+1-((c3BR+mB+1>buf)?buf+1:0))] *((mB-floor(mB))));
					double o3ER = a3ER[(int)(c3ER+mE-((c3ER+mE>buf)?buf+1:0))] *(1.0-(mE-floor(mE)));
					o3ER += (a3ER[(int)(c3ER+mE+1-((c3ER+mE+1>buf)?buf+1:0))] *((mE-floor(mE))));
					double o3HR = a3HR[(int)(c3HR+mH-((c3HR+mH>buf)?buf+1:0))] *(1.0-(mH-floor(mH)));
					o3HR += (a3HR[(int)(c3HR+mH+1-((c3HR+mH+1>buf)?buf+1:0))] *((mH-floor(mH))));

					a3GL[c3GL] = (((o3EL + o3FL) * -2.0) + o3DL);
					a3HL[c3HL] = (((o3DL + o3FL) * -2.0) + o3EL);
					a3IL[c3IL] = (((o3DL + o3EL) * -2.0) + o3FL);
					a3AR[c3AR] = (((o3ER + o3HR) * -2.0) + o3BR);
					a3DR[c3DR] = (((o3BR + o3HR) * -2.0) + o3ER);
					a3GR[c3GR] = (((o3BR + o3ER) * -2.0) + o3HR);

					c3GL++; if (c3GL < 0 || c3GL > buf) c3GL = 0;
					c3HL++; if (c3HL < 0 || c3HL > buf) c3HL = 0;
					c3IL++; if (c3IL < 0 || c3IL > buf) c3IL = 0;
					c3AR++; if (c3AR < 0 || c3AR > buf) c3AR = 0;
					c3DR++; if (c3DR < 0 || c3DR > buf) c3DR = 0;
					c3GR++; if (c3GR < 0 || c3GR > buf) c3GR = 0;

					double o3GL = a3GL[(int)(c3GL+mG-((c3GL+mG>buf)?buf+1:0))] *(1.0-(mG-floor(mG)));
					o3GL += (a3GL[(int)(c3GL+mG+1-((c3GL+mG+1>buf)?buf+1:0))] *((mG-floor(mG))));
					double o3HL = a3HL[(int)(c3HL+mH-((c3HL+mH>buf)?buf+1:0))] *(1.0-(mH-floor(mH)));
					o3HL += (a3HL[(int)(c3HL+mH+1-((c3HL+mH+1>buf)?buf+1:0))] *((mH-floor(mH))));
					double o3IL = a3IL[(int)(c3IL+mI-((c3IL+mI>buf)?buf+1:0))] *(1.0-(mI-floor(mI)));
					o3IL += (a3IL[(int)(c3IL+mI+1-((c3IL+mI+1>buf)?buf+1:0))] *((mI-floor(mI))));
					double o3AR = a3AR[(int)(c3AR+mA-((c3AR+mA>buf)?buf+1:0))] *(1.0-(mA-floor(mA)));
					o3AR += (a3AR[(int)(c3AR+mA+1-((c3AR+mA+1>buf)?buf+1:0))] *((mA-floor(mA))));
					double o3DR = a3DR[(int)(c3DR+mD-((c3DR+mD>buf)?buf+1:0))] *(1.0-(mD-floor(mD)));
					o3DR += (a3DR[(int)(c3DR+mD+1-((c3DR+mD+1>buf)?buf+1:0))] *((mD-floor(mD))));
					double o3GR = a3GR[(int)(c3GR+mG-((c3GR+mG>buf)?buf+1:0))] *(1.0-(mG-floor(mG)));
					o3GR += (a3GR[(int)(c3GR+mG+1-((c3GR+mG+1>buf)?buf+1:0))] *((mG-floor(mG))));

					f3AL = (((o3GR + o3HR) * -2.0) + o3IR);
					f3BL = (((o3GR + o3HR) * -2.0) + o3IR);
					f3CL = (((o3GR + o3HR) * -2.0) + o3IR);

					f3CR = (((o3AL + o3DL) * -2.0) + o3GL);
					f3FR = (((o3AL + o3DL) * -2.0) + o3GL);
					f3IR = (((o3AL + o3DL) * -2.0) + o3GL);

					inputSampleL = (o3GL + o3HL + o3IL)*0.03125;
					inputSampleR = (o3AR + o3DR + o3GR)*0.03125;

					f3AL = (f3AL+f3AL+f3AL+fabs(avg3L))*0.25; avg3L = f3AL;
					f3CR = (f3CR+f3CR+f3CR+fabs(avg3R))*0.25; avg3R = f3CR;

					inputSampleL = fmin(fmax(inputSampleL,-4.0),4.0);
					if (wasPosClipL == true) { //current will be over
						if (inputSampleL<lastSampleL) lastSampleL=0.79+(inputSampleL*0.2);
						else lastSampleL = 0.2+(lastSampleL*0.79);
					} wasPosClipL = false;
					if (inputSampleL>0.99) {wasPosClipL=true;inputSampleL=0.79+(lastSampleL*0.2);}
					if (wasNegClipL == true) { //current will be -over
						if (inputSampleL > lastSampleL) lastSampleL=-0.79+(inputSampleL*0.2);
						else lastSampleL=-0.2+(lastSampleL*0.79);
					} wasNegClipL = false;
					if (inputSampleL<-0.99) {wasNegClipL=true;inputSampleL=-0.79+(lastSampleL*0.2);}
					lastSampleL = inputSampleL;
					inputSampleR = fmin(fmax(inputSampleR,-4.0),4.0);
					if (wasPosClipR == true) { //current will be over
						if (inputSampleR<lastSampleR) lastSampleR=0.79+(inputSampleR*0.2);
						else lastSampleR = 0.2+(lastSampleR*0.79);
					} wasPosClipR = false;
					if (inputSampleR>0.99) {wasPosClipR=true;inputSampleR=0.79+(lastSampleR*0.2);}
					if (wasNegClipR == true) { //current will be -over
						if (inputSampleR > lastSampleR) lastSampleR=-0.79+(inputSampleR*0.2);
						else lastSampleR=-0.2+(lastSampleR*0.79);
					} wasNegClipR = false;
					if (inputSampleR<-0.99) {wasNegClipR=true;inputSampleR=-0.79+(lastSampleR*0.2);}
					lastSampleR = inputSampleR;

					inputSampleL = asin(inputSampleL*0.7);
					inputSampleR = asin(inputSampleR*0.7);

					bez[bez_CL] = bez[bez_BL];
					bez[bez_BL] = bez[bez_AL];
					bez[bez_AL] = inputSampleL;
					bez[bez_SampL] = 0.0;

					bez[bez_CR] = bez[bez_BR];
					bez[bez_BR] = bez[bez_AR];
					bez[bez_AR] = inputSampleR;
					bez[bez_SampR] = 0.0;
				}

				double X = bez[bez_cycle] * bezTrim;
				double CBL = (bez[bez_CL]*(1.0-X))+(bez[bez_BL]*X);
				double CBR = (bez[bez_CR]*(1.0-X))+(bez[bez_BR]*X);
				double BAL = (bez[bez_BL]*(1.0-X))+(bez[bez_AL]*X);
				double BAR = (bez[bez_BR]*(1.0-X))+(bez[bez_AR]*X);
				double CBAL = (bez[bez_BL]+(CBL*(1.0-X))+(BAL*X))*-0.25;
				double CBAR = (bez[bez_BR]+(CBR*(1.0-X))+(BAR*X))*-0.25;
				inputSampleL = CBAL+bez[bez_AvgOutSampL]; bez[bez_AvgOutSampL] = CBAL;
				inputSampleR = CBAR+bez[bez_AvgOutSampR]; bez[bez_AvgOutSampR] = CBAR;

				if (out != 1.0) {
					inputSampleL *= out;
					inputSampleR *= out;
				}

				if (wet != 1.0) {
					inputSampleL = (inputSampleL * wet) + (drySampleL * (1.0-wet));
					inputSampleR = (inputSampleR * wet) + (drySampleR * (1.0-wet));
				}
				fpdL ^= fpdL << 13; fpdL ^= fpdL >> 17; fpdL ^= fpdL << 5;
				fpdR ^= fpdR << 13; fpdR ^= fpdR >> 17; fpdR ^= fpdR << 5;

				*out1 = inputSampleL;
				*out2 = inputSampleR;

				in1++;
				in2++;
				out1++;
				out2++;
			}
		}

    private:

    	float A;
    	float B;
    	float C;
    	float D;
    	float E;
    	float F;
    	float G;
    	float H;

    	double sampleRate {44100};

    	double getSampleRate() const
    	{
    		return sampleRate;
    	}

    	double a3AL[32767+5];
    	double a3BL[32767+5];
    	double a3CL[32767+5];
    	double a3DL[32767+5];
    	double a3EL[32767+5];
    	double a3FL[32767+5];
    	double a3GL[32767+5];
    	double a3HL[32767+5];
    	double a3IL[32767+5];
    	double a3AR[32767+5];
    	double a3BR[32767+5];
    	double a3CR[32767+5];
    	double a3DR[32767+5];
    	double a3ER[32767+5];
    	double a3FR[32767+5];
    	double a3GR[32767+5];
    	double a3HR[32767+5];
    	double a3IR[32767+5];

    	int c3AL ;
    	int c3AR ;
    	int c3BL ;
    	int c3BR ;
    	int c3CL ;
    	int c3CR ;
    	int c3DL ;
    	int c3DR ;
    	int c3EL ;
    	int c3ER ;
    	int c3FL ;
    	int c3FR ;
    	int c3GL ;
    	int c3GR ;
    	int c3HL ;
    	int c3HR ;
    	int c3IL ;
    	int c3IR ;

    	double f3AL ;
    	double f3BL ;
    	double f3CL ;
    	double f3CR ;
    	double f3FR ;
    	double f3IR	;
    	double avg3L;
    	double avg3R;

    	template <typename SmootherType>
		void updateSmoother(SmootherType& smoother, int numSamples)
    	{
            // Get start values
    		const float startA = smoother.getSpeed();
    		const float startB = smoother.getRando();
    		const float startC = smoother.getDepth();
    		const float startD = smoother.getRegen();
    		const float startE = smoother.getDerez();
    		const float startF = smoother.getBuffer();
    		const float startG = smoother.getOutput();
    		const float startH = smoother.getDryWet();

            // Non-interpolated parameters use start values
            A = startA;
            B = startB;
            C = startC;
            D = startD;
            F = startF;

    		if (numSamples > 1)
            {
                if (numSamples > 2)
    			    smoother.setSmoother(numSamples - 2, SmootherType::SmootherUpdateMode::liveInRealTime);

                // Advance all smoothers to the end of the block to maintain sync
                (void)smoother.getSpeed();
                (void)smoother.getRando();
                (void)smoother.getDepth();
                (void)smoother.getRegen();
                E = smoother.getDerez(); // Interpolated target uses end value
                (void)smoother.getBuffer();
                G = smoother.getOutput(); // Interpolated target uses end value
                H = smoother.getDryWet(); // Interpolated target uses end value
            }
            else
            {
                E = startE;
                G = startG;
                H = startH;
            }
    	}

    	enum
    	{
    		bez_AL,
			bez_AR,
			bez_BL,
			bez_BR,
			bez_CL,
			bez_CR,
			bez_InL,
			bez_InR,
			bez_UnInL,
			bez_UnInR,
			bez_SampL,
			bez_SampR,
			bez_AvgInSampL,
			bez_AvgInSampR,
			bez_AvgOutSampL,
			bez_AvgOutSampR,
			bez_cycle,
			bez_total
		};

    	double bez[bez_total];

    	double rotate;
    	double oldfpd;

    	int buf {};

    	double vibDepth;
    	double derezA  ;
    	double derezB  ;
    	double outA    ;
    	double outB    ;
    	double wetA    ;
    	double wetB    ;

    	double lastSampleL;
    	bool wasPosClipL  ;
    	bool wasNegClipL  ;
    	double lastSampleR;
    	bool wasPosClipR  ;
    	bool wasNegClipR  ;

    	uint32_t fpdL;
    	uint32_t fpdR;

    };
}