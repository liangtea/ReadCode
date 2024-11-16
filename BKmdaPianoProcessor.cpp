/*
 *  mdaPianoProcessor.cpp
 *  mda-vst3
 *
 *  Created by Arne Scheffler on 6/14/08.
 *
 *  mda VST Plug-ins
 *
 *  Copyright (c) 2008 Paul Kellett
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*在 Cubase 中按下音符时，以下是主要的工作流程：

noteEvent 函数负责处理音符按下事件，初始化和设置音符的相关参数。

doProcessing 函数在每个音频缓冲周期被调用，生成实际的 PCM 数据。

生成的 PCM 数据会被传递回 Cubase，进行进一步处理并播放。

Envelope Decay (50.00%):

作用: 控制音符在松开琴键后音量衰减的速度。值越大，衰减越慢。

Envelope Release (50.00%):

作用: 控制音符在松开琴键后音量完全消失的时间。值越大，释放时间越长。

Hardness Offset (50.00%):

作用: 调整音符的硬度或音色的明亮度。值越高，音色越硬或越亮。

Velocity to Hardness (50.00%):

作用: 控制音符触发速度对硬度的影响。值越高，演奏速度对音符硬度的影响越大。

Muffling Filter (80.30%):

作用: 调节音符的闷音效果。值越高，音符听起来越柔和或低沉。

Velocity to Muffling (25.10%):

作用: 控制音符触发速度对闷音效果的影响。值越高，演奏速度对闷音效果的影响越大。

Velocity Sensitivity (37.60%):

作用: 控制音符触发速度对音量的影响。值越高，演奏速度对音量的影响越大。

Stereo Width (100.00%):

作用: 调节立体声宽度。值越高，立体声效果越明显。

Polyphony (15%):

作用: 控制最大复音数，即同时可以播放的音符数量。

Fine Tuning (50.00%):

作用: 调整音符的微调音高。用于精细调节音准。

Random Detuning (24.60%):

作用: 增加音符的随机失调效果，模拟复古音色或增加音色丰富性。

Stretch Tuning (0.00%):

作用: 调整音符的拉伸调音效果，适用于特定的调音需求。

Mod Wheel (0.0000):

作用: 控制调制轮的效果，常用于调整音色的特定特性（如颤音、滤波等）。

Sustain (有一个开关图标):

作用: 控制延音踏板的状态。开启时，按下的音符会延音，直到踏板释放。

*/

#include "mdaPianoProcessor.h"
#include "mdaPianoController.h"
#include "mdaPianoData.h"

#include <cstdio>
#include <cmath>

namespace Steinberg {
namespace Vst {
namespace mda {

#define SILENCE 0.0001f  //voice choking

//-----------------------------------------------------------------------------
float PianoProcessor::programParams[][NPARAMS] = { 
	{0.500f, 0.500f, 0.500f, 0.5f, 0.803f, 0.251f, 0.376f, 0.500f, 0.330f, 0.500f, 0.246f, 0.500f},
	{0.500f, 0.500f, 0.500f, 0.5f, 0.751f, 0.000f, 0.452f, 0.000f, 0.000f, 0.500f, 0.000f, 0.500f},
	{0.902f, 0.399f, 0.623f, 0.5f, 1.000f, 0.331f, 0.299f, 0.499f, 0.330f, 0.500f, 0.000f, 0.500f},
	{0.399f, 0.251f, 1.000f, 0.5f, 0.672f, 0.124f, 0.127f, 0.249f, 0.330f, 0.500f, 0.283f, 0.667f},
	{0.648f, 0.500f, 0.500f, 0.5f, 0.298f, 0.602f, 0.550f, 0.850f, 0.356f, 0.500f, 0.339f, 0.660f},
	{0.500f, 0.602f, 0.000f, 0.5f, 0.304f, 0.200f, 0.336f, 0.651f, 0.330f, 0.500f, 0.317f, 0.500f},
	{0.450f, 0.598f, 0.626f, 0.5f, 0.603f, 0.500f, 0.174f, 0.331f, 0.330f, 0.500f, 0.421f, 0.801f},
	{0.050f, 0.957f, 0.500f, 0.5f, 0.299f, 1.000f, 0.000f, 0.500f, 0.330f, 0.450f, 0.718f, 0.000f},
};

//-----------------------------------------------------------------------------
PianoProcessor::PianoProcessor ()
: currentProgram (0)
{
	setControllerClass (PianoController::uid);
	allocParameters (NPARAMS);
}

//-----------------------------------------------------------------------------
PianoProcessor::~PianoProcessor ()
{
}

//-----------------------------------------------------------------------------
tresult PLUGIN_API PianoProcessor::initialize (FUnknown* context)
{
	tresult res = Base::initialize (context);
	if (res == kResultTrue)
	{
		addEventInput (USTRING("MIDI in"), 1);
		addAudioOutput (USTRING("Stereo Out"), SpeakerArr::kStereo);
		// Fs设置采样率为44.1 kHz	
		// iFs计算采样时间间隔（采样周期），即1秒内采样的次数的倒数
		// cmax设定最大音量级别为127（标准MIDI最大值）
		Fs = 44100.0f;  iFs = 1.0f/Fs;  cmax = 0x7F;  //just in case...
		//waves：这是一个指向波形数据的指针。pianoData：包含钢琴音色的波形数据。这行代码将 waves 指针初始化为指向 pianoData，从而在后续处理中使用这些波形数据来生成钢琴音色。
		waves = pianoData;
		/*存储在 pianoData(mdaPianoData.h) 数组中的音频样本。这些样本以 PCM 格式存储，用于再现钢琴的声音。每个样本值代表一个特定时间点上的音频信号的振幅
		PCM（脉冲编码调制）数据是将模拟音频信号数字化的结果。在这种情况下，数组 pianoData 包含了短整型（short）值，这些值交替表示正和负的振幅，形成音频波形。
		正值和负值：每个值表示在该采样点上的振幅（即音频信号的强度）。正值和负值表示振幅相对于中线的偏移方向。
		动态变化：这些数据会随着时间变化，表示声音的振动和波动。相邻的值之间的变化构成了音频信号的波形。
		使用这些数据再现声音，在合成器或音频处理器中，这些PCM数据将通过DAC（数模转换器）转换回模拟信号，驱动扬声器再现声音。
		*/
		
		//Waveform data and keymapping is hard-wired in *this* version
		
		/*
		这段代码定义了钢琴键盘上的各个键组（keygroup）的波形数据和键映射。每个键组包含以下信息：
		root：根音，即该键组的基本音高（MIDI音符编号）。
		high：该键组能处理的最高音高（MIDI音符编号）。
		pos：该键组在波形数据数组中的起始位置。
		end：该键组在波形数据数组中的结束位置。
		loop：该键组在波形数据数组中的循环位置。
		每个键组（如 kgrp[0]）包含五个字段：
		root：kgrp[0].root = 36 表示该键组的根音为 MIDI 音符编号 36（对应音符 C2）。
		high：kgrp[0].high = 37 表示该键组能够处理的最高音高为 MIDI 音符编号 37（对应音符 C#2）。
		pos：kgrp[0].pos = 0 表示波形数据数组的起始位置，从第 0 个样本开始。
		end：kgrp[0].end = 36275 表示波形数据数组的结束位置，到第 36275 个样本结束。
		loop：kgrp[0].loop = 14774 表示波形数据在第 14774 个样本处开始循环。
		键组的作用
		这些键组定义了不同音高的音符如何映射到波形数据。这在合成器处理中非常重要，因为它允许合成器根据接收到的 MIDI 音符事件，选择适当的波形数据来生成相应的声音。
		当合成器接收到一个 MIDI 音符事件时，例如，按下键盘上的 C2 音符（MIDI 音符编号 36），合成器会查找对应的键组（kgrp[0]），并使用该键组的波形数据（从 pos = 0 到 end = 36275，并在 loop = 14774 处循环）来生成该音符的声音。
		在音频合成中，循环用于重复播放一段音频数据，以延长音符的持续时间，而无需占用大量存储空间。这对于保持音符的持续声音（例如钢琴音符的延音部分）特别有用。怎么循环法？当一个音符被按下时，合成器会从波形数据的起始位置 pos 开始播放，直到达到 end 位置。当达到 end 位置时，如果设定了循环，播放会跳回到 loop 位置继续播放，这样就可以持续生成声音，直到音符被释放。这是一个简单的示例：if (V->pos > V->end) V->pos -= V->loop;
		这行代码检查当前的播放位置 pos 是否超过了结束位置 end，如果超过了，就将 pos 减去 loop 位置，从而实现循环。
		第一个键组 (kgrp[0]) 的根音是 MIDI 音符 36，对应的音符是 C2。
		它的最高音是 MIDI 音符 37，对应的音符是 C#2。
		这样，一个键组就覆盖了两个音符（C2 和 C#2）
		合成器通过计算来决定每个音符应该使用哪个键组。在接收到 MIDI 音符时，会遍历 kgrp 数组，找到适合的键组。
		通过 noteEvent 函数来处理新的音符事件。
		合成器会根据音符的音高，查找适合的键组。例如：
		k = 0;
		while (note > kgrp[k].high) k++;
		这种查找确保了每个音符都能找到相应的键组。
		查找到键组后，合成器会使用该键组的波形数据来生成音符。
		每个键组的 root 和 high 确保了一定范围内的音符都能被覆盖。
		虽然 kgrp 数组只有 15 个键组，但因为每个键组可以覆盖多个音符，最终可以涵盖整个钢琴的 88 个音符。例如，以下是简化的键组覆盖范围：
		kgrp[0]  // C2, C#2
		kgrp[1]  // E2, F2
		kgrp[2]  // G2, A2, Bb2
		kgrp[3]  // C3, C#3
		kgrp[4]  // E3, F3
		kgrp[5]  // G3, A3, Bb3
		// ... 依次类推
		每个键组覆盖2到3个音符，通过这种方式，15个键组就可以覆盖整个钢琴的88个音符。
		*/
		kgrp[ 0].root = 36;  kgrp[ 0].high = 37;  kgrp[ 0].pos = 0;       kgrp[ 0].end = 36275;   kgrp[ 0].loop = 14774;
		kgrp[ 1].root = 40;  kgrp[ 1].high = 41;  kgrp[ 1].pos = 36278;   kgrp[ 1].end = 83135;   kgrp[ 1].loop = 16268;
		kgrp[ 2].root = 43;  kgrp[ 2].high = 45;  kgrp[ 2].pos = 83137;   kgrp[ 2].end = 146756;  kgrp[ 2].loop = 33541;
		kgrp[ 3].root = 48;  kgrp[ 3].high = 49;  kgrp[ 3].pos = 146758;  kgrp[ 3].end = 204997;  kgrp[ 3].loop = 21156;
		kgrp[ 4].root = 52;  kgrp[ 4].high = 53;  kgrp[ 4].pos = 204999;  kgrp[ 4].end = 244908;  kgrp[ 4].loop = 17191;
		kgrp[ 5].root = 55;  kgrp[ 5].high = 57;  kgrp[ 5].pos = 244910;  kgrp[ 5].end = 290978;  kgrp[ 5].loop = 23286;
		kgrp[ 6].root = 60;  kgrp[ 6].high = 61;  kgrp[ 6].pos = 290980;  kgrp[ 6].end = 342948;  kgrp[ 6].loop = 18002;
		kgrp[ 7].root = 64;  kgrp[ 7].high = 65;  kgrp[ 7].pos = 342950;  kgrp[ 7].end = 391750;  kgrp[ 7].loop = 19746;
		kgrp[ 8].root = 67;  kgrp[ 8].high = 69;  kgrp[ 8].pos = 391752;  kgrp[ 8].end = 436915;  kgrp[ 8].loop = 22253;
		kgrp[ 9].root = 72;  kgrp[ 9].high = 73;  kgrp[ 9].pos = 436917;  kgrp[ 9].end = 468807;  kgrp[ 9].loop = 8852;
		kgrp[10].root = 76;  kgrp[10].high = 77;  kgrp[10].pos = 468809;  kgrp[10].end = 492772;  kgrp[10].loop = 9693;
		kgrp[11].root = 79;  kgrp[11].high = 81;  kgrp[11].pos = 492774;  kgrp[11].end = 532293;  kgrp[11].loop = 10596;
		kgrp[12].root = 84;  kgrp[12].high = 85;  kgrp[12].pos = 532295;  kgrp[12].end = 560192;  kgrp[12].loop = 6011;
		kgrp[13].root = 88;  kgrp[13].high = 89;  kgrp[13].pos = 560194;  kgrp[13].end = 574121;  kgrp[13].loop = 3414;
		kgrp[14].root = 93;  kgrp[14].high = 999; kgrp[14].pos = 574123;  kgrp[14].end = 586343;  kgrp[14].loop = 2399;

		//initialise...
		//synthData.numVoices=32，也就是最大32个复音。合成器可以在同一时间播放32个不同的音符。
		for(int32 v=0; v<synthData.numVoices; v++) 
		{
			memset (&synthData.voice[v], 0, sizeof (VOICE));
			//将每个声部的包络（envelope）初始化为 0。这表示这些声部目前没有任何激活的音符。
			synthData.voice[v].env = 0.0f;
			//设置衰减参数，使所有音符的声音逐渐衰减，直至完全停止。这是为了确保在合成器初始化时，没有任何未结束的音符在播放。
			synthData.voice[v].dec = 0.99f; //all notes off
		}
		//将音量设置为0.2。这是一个初始值，用于控制合成器输出音频的整体音量。
		volume = 0.2f;
		//这行代码将muff（可能是“muffle”的缩写）设置为160.0。它通常用于控制滤波器参数，使声音变得更加柔和或模糊。
		muff = 160.0f;
		//cpos可能是一个环形缓冲区的当前位置，用于处理音频数据。
		//synthData.sustain：用于表示当前的延音状态，0表示没有延音。
		//synthData.activevoices：表示当前活跃的声部（voices）数量，0表示没有活跃的声部。
		cpos = synthData.sustain = synthData.activevoices = 0;
		//这行代码为一个浮点型数组 comb 分配256个元素的内存空间。这个数组可能用于存储和处理音频信号，例如用于延迟效果或滤波器。
		comb = new float[256];

		/*
		NPARAMS 表示参数的数量。
		params[i] 是参数数组的第 i 个元素。
		programParams[0][i] 是 programParams 数组的第0个程序的第 i 个参数值。
		也就是初始化第一个program的参数。
		*/
		for (int32 i = 0; i < NPARAMS; i++)
			params[i] = programParams[0][i];

		recalculate ();
	}
	return res;
}

//-----------------------------------------------------------------------------
tresult PLUGIN_API PianoProcessor::terminate ()
{
	if (comb) delete[] comb;
	comb = nullptr;
	return Base::terminate ();
}

//-----------------------------------------------------------------------------
tresult PLUGIN_API PianoProcessor::setActive (TBool state)
{
	//TBool state 是函数参数，表示插件的激活状态。true 表示激活，false 表示停用。
	if (state)
	{
		synthData.init ();
		Fs = getSampleRate ();
		iFs = 1.0f / Fs;
		/*
		根据采样率调整 cmax 参数：如果采样率大于 64 kHz，设置 cmax 为 0xFF（255）。否则，设置 cmax 为 0x7F（127）。
		*/
		if (Fs > 64000.0f) cmax = 0xFF; else cmax = 0x7F;
		//将 comb 数组的内存清零。comb 数组的大小为 256 个浮点数，用于存储一些音频信号数据（例如延迟或滤波器的数据）。
		memset (comb, 0, sizeof (float) * 256);
	}
	else
		allNotesOff ();
	return Base::setActive (state);
}

//-----------------------------------------------------------------------------
void PianoProcessor::setParameter (ParamID index, ParamValue newValue, int32 sampleOffset)
{
	/*
	ParamID index 是参数索引，用于指定要设置的参数。
	ParamValue newValue 是新的参数值。
	int32 sampleOffset 是样本偏移量，通常用于延迟设置参数的生效时间。
	*/
	//如果 index 小于 NPARAMS（参数总数），则调用基类的 setParameter 方法设置参数。
	if (index < NPARAMS)
		Base::setParameter (index, newValue, sampleOffset);
	else if (index == BaseController::kPresetParam) // program change
	{
		/*
		如果 index 是预设参数（kPresetParam），则根据 newValue 计算新的程序（预设）索引，并将 currentProgram 设置为该索引。
		从 programParams 数组中获取新预设参数，并更新 params 数组中的所有参数值。
		newValue * kNumPrograms = 0.75 * 10 = 7.5
		(int32)(newValue * kNumPrograms) = (int32)7.5 = 7
		std::min<int32>(9, 7) = 7
		在这种情况下，currentProgram 将被设置为 7。

		*/
		currentProgram = std::min<int32> (kNumPrograms - 1, (int32)(newValue * kNumPrograms));
		const float* newParams = programParams[currentProgram];
		if (newParams)
		{
			for (int32 i = 0; i < NPARAMS; i++)
				params[i] = newParams[i];
		}
	}
	else if (index == BaseController::kModWheelParam) // mod wheel
	{
		/*
		如果 index 是调制轮参数（kModWheelParam），则将 newValue 乘以 127。计算 muff 参数，用于调整滤波器或其他音频效果。
		*/
		newValue *= 127.;
		muff = 0.01f * (float)((127 - newValue) * (127 - newValue));
	}
	else if (index == BaseController::kSustainParam)
	{
		/*
		如果 index 是延音踏板参数（kSustainParam），则将 synthData.sustain 设置为 newValue 是否大于 0.5。
		如果 synthData.sustain 设置为 0，则遍历所有声部（voices），找到那些延音音符（SustainNoteID），并更新其衰减值 v.dec。
		*/
		synthData.sustain = newValue > 0.5;
		if (synthData.sustain==0)
		{
			for (auto& v : synthData.voice)
			{
				if (v.noteID = SustainNoteID)
				{
					/*
					如果当前音符处于延音状态，代码会计算并设置衰减参数 dec。具体计算公式如下：iFs 是采样时间间隔，即采样率的倒数。
					(double)v.note 将音符值转换为双精度浮点数。params[1] 是一个外部参数，影响衰减时间。
					计算公式包含两个 exp 函数（指数函数），用于计算复合衰减率。
					*/
					v.dec = (float)exp (-iFs * exp (6.0 + 0.01 * (double)v.note - 5.0 * params[1]));
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------
void PianoProcessor::setCurrentProgram (Steinberg::uint32 val)
{
	currentProgram = val;
}

//-----------------------------------------------------------------------------
void PianoProcessor::setCurrentProgramNormalized (ParamValue val)
{
	setCurrentProgram (std::min<int32> (kNumPrograms - 1, (int32)(val * kNumPrograms)));
}

//-----------------------------------------------------------------------------
void PianoProcessor::doProcessing (ProcessData& data)
{
	/*这个函数负责处理音频数据，是音频合成器的核心处理循环。它处理合成器的所有音频处理任务，包括音符的生成和混合。让我们逐行解析这个函数的作用。
	获取处理的数据帧数 sampleFrames。
	获取输出缓冲区的指针 out0 和 out1，分别对应左右声道。
	初始化一些局部变量，用于音频处理循环。
	*/
	int32 sampleFrames = data.numSamples;
	
	float* out0 = data.outputs[0].channelBuffers32[0];
	float* out1 = data.outputs[0].channelBuffers32[1];

	int32 frame=0, frames, v;
	float x, l, r;
	int32 i;

	synthData.eventPos = 0;
	/*
	主循环中，遍历每个样本帧。内部循环处理每个活跃的声部（voice），计算每个声部的音频数据，并将其混合到左右声道。调用 noteEvent 处理 MIDI 事件，更新声部状态。
	样本帧（Sample Frame）样本帧是指一组采样点。在单声道音频中，每个样本帧包含一个样本；而在立体声音频中，每个样本帧包含两个样本，一个用于左声道（left channel），一个用于右声道（right channel）。
	在立体声音频中，如果有 44100 个样本帧，那么实际上有 88200 个样本，因为每个样本帧包含左右两个声道的样本。

	*/
	if (synthData.activevoices > 0 || synthData.hasEvents ())
	{    
		//如果有活跃的声部或待处理的事件，则进入处理逻辑。
		while (frame<sampleFrames)
		{/*frame 是当前处理的样本帧索引。sampleFrames 是总的样本帧数。循环遍历每个样本帧。*/

			/*获取当前事件的样本偏移量 sampleOffset。确保 frames 不超过总样本帧数 sampleFrames。更新当前样本帧索引。*/
			frames = synthData.events[synthData.eventPos].sampleOffset;
			if (frames>sampleFrames) frames = sampleFrames;
			frames -= frame;
			frame += frames;

			while (--frames>=0)
			{/*处理当前样本帧的循环。*/

				//初始化左右声道的音频信号：
				VOICE *V = synthData.voice.data ();
				l = r = 0.0f;

				for(v=0; v<synthData.activevoices; v++)
				{/*遍历每个活跃的声部。*/
					// 处理每个活跃的声部（voice）计算音频信号，并累积到左声道和右声道
					//integer-based linear interpolation
					//线性插值：解决分辨率问题：由于波形数据是以离散样本存储的，当需要在不同频率上播放音符时，可能会出现样本点之间的跳跃。
					//线性插值通过计算样本点之间的中间值，平滑过渡。避免音频失真：如果不进行插值，音频信号会显得生硬或“步进化”，从而引起失真。
					//通过线性插值，合成器能够平滑地播放音符，而不会在样本点之间产生突然的变化。
					//下面的waves就是PCM音频数据
					V->frac += V->delta;  // 增加小数部分 
					V->pos += V->frac >> 16;// 更新位置
					V->frac &= 0xFFFF;// 保留小数部分
					if (V->pos > V->end) V->pos -= V->loop;// 循环处理
					//i = (i << 7) + (V->frac >> 9) * (waves[V->pos + 1] - i) + 0x40400000;   //not working on intel mac !?!
					i = waves[V->pos] + ((V->frac * (waves[V->pos + 1] - waves[V->pos])) >> 16);// 线性插值
					x = V->env * (float)i / 32768.0f;
					//x = V->env * (*(float *)&i - 3.0f);  //fast int->float

					//包络和滤波：控制音符的动态变化：包络用于控制音符的音量随时间的变化，从而模拟真实乐器的演奏特性。
					//例如，钢琴音符通常有一个快速的音量上升（攻击），然后逐渐减弱（衰减和释放）。
					//改变音色：滤波器用于改变音频信号的频谱特性，模仿不同的音色。
					//例如，可以通过滤波器来模拟钢琴的音色特征，使得声音更加真实和具有表现力。
					//通过包络和滤波，可以控制音符的动态变化和音色，使得合成器生成的声音更加自然和富有表现力。
					V->env = V->env * V->dec;  //envelope
					V->f0 += V->ff * (x + V->f1 - V->f0);  //muffle filter
					V->f1 = x;


					//累加左右声道的音频信号：生成立体声音效：在立体声系统中，左右声道的音频信号需要分别计算和输出，以创造空间感和方向感。
					//例如，某些音符可能在左声道上音量更大，而在右声道上音量较小，从而模拟声音来源的方向。
					//混合多个声部：同时播放多个音符时，需要将每个声部的音频信号累加，以生成最终的输出音频信号。
					//通过累加各个声部的音频信号，合成器能够生成复杂的音频输出，同时处理多个音符并生成立体声音效。
					l += V->outl * V->f0;
					r += V->outr * V->f0;

					//防止异常值：
					if (!(l > -2.0f) || !(l < 2.0f))
					{
						printf ("what is this?   %d,  %f,  %f\n", i, x, V->f0);
						l = 0.0f;
					}  
					if (!(r > -2.0f) || !(r < 2.0f))
					{
						r = 0.0f;
					}  

					V++;
				}
				//立体声模拟器：
				comb[cpos] = l + r;
				++cpos &= cmax;
				x = cdep * comb[cpos];  //stereo simulator

				*out0++ = l + x;// 输出到左声道
				*out1++ = r - x;// 输出到右声道
			}

			if (frame<sampleFrames)
			{/*处理事件：*/
				noteEvent (synthData.events[synthData.eventPos]);
				++synthData.eventPos;
			}
		}
	}
	/*这行代码使用一个 for 循环，从 0 到 synthData.activevoices - 1 遍历所有活跃的声部。synthData.activevoices 表示当前活跃的声部数量。*/
	for(v=0; v<synthData.activevoices; v++) 
		/*这行代码检查当前声部的包络值（env）是否小于 SILENCE。
		SILENCE 是一个预定义的常量（在上文提到 #define SILENCE 0.0001f），表示静默阈值。如果包络值小于这个阈值，说明该声部的音量已经很小，接近静默状态。*/
		if (synthData.voice[v].env < SILENCE) 
			/*--synthData.activevoices：先将 synthData.activevoices 减 1，表示减少一个活跃声部。
			synthData.voice[v] = synthData.voice[synthData.activevoices]：将最后一个活跃声部的数据复制到当前声部位置，以覆盖当前处于静默状态的声部。
			通过以上步骤，代码移除了当前活跃声部列表中处于静默状态的声部，并确保剩余的活跃声部被正确地管理和更新。*/
			synthData.voice[v] = synthData.voice[--synthData.activevoices];
}

//-----------------------------------------------------------------------------
void PianoProcessor::noteEvent (const Event& event)
{
	/*这个函数负责处理 MIDI 事件，包括音符的按下和释放。当接收到 MIDI 事件时，合成器需要相应地生成或停止音符。
	先处理MIDI事件：当MIDI事件到达时，系统会立即调用 noteEvent 函数，以确保合成器的状态及时更新。这意味着noteEvent 会在 doProcessing 之前被执行。
	随后处理音频缓冲：在处理MIDI事件之后，音频引擎会在下一个音频缓冲周期调用 doProcessing 函数，以生成音频输出。
	*/
	float l=99.0f;
	int32  v, vl=0, k, s;

	if (event.type == Event::kNoteOnEvent)
	{/*NoteOn 事件处理：当接收到 NoteOn 事件时，生成新的音符，或在活跃声部已满时，取代音量最小的活跃声部。
	 NoteOff 事件处理：当接收到 NoteOff 事件时，停止相应的音符。
	 音符按下：当您在MIDI编辑器中添加或拖拉一个音符时，这相当于在实际演奏中按下该音符。这个动作生成一个 NoteOn 事件，告诉合成器需要开始播放该音符。
	 例如，当您在钢琴卷帘窗口中绘制或拉长一个音符时，Cubase会在该音符开始的位置生成一个 NoteOn 事件。
	 音符释放：当您在MIDI编辑器中缩短或删除一个音符时，这相当于在实际演奏中释放该音符。这个动作生成一个 NoteOff 事件，告诉合成器需要停止播放该音符。
	 例如，当您在钢琴卷帘窗口中删除或缩短一个音符时，Cubase会在该音符结束的位置生成一个 NoteOff 事件。
	 
	 */
		auto& noteOn = event.noteOn;
		auto note = noteOn.pitch;
		float velocity = noteOn.velocity * 127;

		/*添加或替换活跃声部：*/
		if (synthData.activevoices < poly) //add a note
		{
			vl = synthData.activevoices;
			synthData.activevoices++;
		}
		else //steal a note
		{
			for(v=0; v<poly; v++)  //find quietest voice
			{
				if (synthData.voice[v].env < l) { l = synthData.voice[v].env;  vl = v; }
			}
		}

		//计算调音参数：
		k = (note - 60) * (note - 60);
		l = fine + random * ((float)(k % 13) - 6.5f);  //random & fine tune
		if (note > 60) l += stretch * (float)k; //stretch

		//调整尺寸参数：
		s = size;
		if (velocity > 40) s += (int32)(sizevel * (float)(velocity - 40));  


		//查找键组：
		k = 0;
		while (note > (kgrp[k].high + s)) k++;  //find keygroup

		//计算频率和波形位置：
		l += (float)(note - kgrp[k].root); //pitch
		l = 22050.0f * iFs * (float)exp (0.05776226505 * l);
		synthData.voice[vl].delta = (int32)(65536.0f * l);
		synthData.voice[vl].frac = 0;
		synthData.voice[vl].pos = kgrp[k].pos;
		synthData.voice[vl].end = kgrp[k].end;
		synthData.voice[vl].loop = kgrp[k].loop;

		//设置包络和滤波参数：
		synthData.voice[vl].env = (0.5f + velsens) * (float)pow (0.0078f * velocity, velsens); //velocity
		l = 50.0f + params[4] * params[4] * muff + muffvel * (float)(velocity - 64); //muffle
		if (l < (55.0f + 0.25f * (float)note)) l = 55.0f + 0.25f * (float)note;
		if (l > 210.0f) l = 210.0f;
		synthData.voice[vl].ff = l * l * iFs;
		synthData.voice[vl].f0 = synthData.voice[vl].f1 = 0.0f;


		//设置音符和立体声输出参数：
		synthData.voice[vl].note = note; //note->pan
		if (note <  12) note = 12;
		if (note > 108) note = 108;
		l = volume * trim;
		synthData.voice[vl].outr = l + l * width * (float)(note - 60);
		synthData.voice[vl].outl = l + l - synthData.voice[vl].outr;

		//设置衰减参数：
		if (note < 44) note = 44; //limit max decay length
		l = 2.0f * params[0];
		if (l < 1.0f) l += 0.25f - 0.5f * params[0];
		synthData.voice[vl].dec = (float)exp (-iFs * exp (-0.6 + 0.033 * (double)note - l));
		synthData.voice[vl].noteID = noteOn.noteId;
	}
	else //note off
	{
		auto& noteOff = event.noteOff;
		auto note = noteOff.pitch;
		for(v=0; v<synthData.numVoices; v++) if (synthData.voice[v].noteID==noteOff.noteId) //any voices playing that note?
		{
			if (synthData.sustain==0)
			{
				if (note < 94) //no release on highest notes
				synthData.voice[v].dec = (float)exp (-iFs * exp (2.0 + 0.017 * (double)note - 2.0 * params[1]));
			}
			else synthData.voice[v].noteID = SustainNoteID;
		}
	}
	//### 总结 
	//- **NoteOn 事件**：增加或替换活跃声部，计算和设置相关参数（频率、波形、包络、滤波等），并初始化音符相关的内部状态。 
	//- **NoteOff 事件**：停止相应音符，根据延音踏板状态决定是否立即停止还是延音。 
	//这段代码确保了在接收到 MIDI 事件时，合成器能够正确生成或停止音符，实现流畅的音频播放和控制。
}

//-----------------------------------------------------------------------------
void PianoProcessor::preProcess ()
{
	synthData.clearEvents ();
}

//-----------------------------------------------------------------------------
void PianoProcessor::processEvent (const Event& e)
{
	synthData.processEvent (e);
}

//-----------------------------------------------------------------------------
void PianoProcessor::allNotesOff ()
{
  for (int32 v=0; v<synthData.numVoices; v++) synthData.voice[v].dec=0.99f;
  synthData.sustain = 0;
  muff = 160.0f;
}

//-----------------------------------------------------------------------------
void PianoProcessor::recalculate ()
{
	/*
	recalculate 函数根据一组参数 (params) 重新计算和设置钢琴合成器的各种内部参数，这些参数直接影响到声音的生成和处理方式。通过调整这些参数，用户可以定制合成器的声音特性，比如音量、速度敏感度、滤波效果和复音数等。
	
	size
	作用：根据 params[2] 计算大小参数 size。
	解释：这个参数可能用于控制键组的大小或音高范围。将 params[2] 乘以 12 然后减去 6，结果转换为整数类型。
	
	sizevel
	作用：根据 params[3] 计算大小速度关系参数 sizevel。
	解释：这个参数可能用于控制与音量或速度相关的大小调整。
	
	muffvel
	作用：根据 params[5] 计算滤波器速度参数 muffvel。
	解释：这个参数用于控制滤波效果，平方后乘以 5.0。
	
	velsens
	作用：计算速度敏感度参数 velsens。
	解释：将 params[6] 加倍再加 1.0。如果 params[6] 小于 0.25，进一步调整。
	
	fine
	作用：计算微调参数 fine。
	解释：这个参数用于微调音调，结果为 params[9] 减去 0.5。
	
	random
	作用：计算随机调节参数 random。
	解释：将 params[10] 的平方乘以 0.077
	
	stretch
	作用：计算音高拉伸参数 stretch。
	解释：将 params[11] 减去 0.5，然后乘以 0.000434。
	
	cdep trim
	作用：计算立体声模拟器的深度 cdep 和修剪参数 trim。
	解释：将 params[7] 平方得到 cdep，修剪参数 trim 是 1.50 减去 cdep 乘以 0.79。
	
	width
	作用：计算立体声宽度参数 width。
	解释：将 params[7] 乘以 0.04。如果结果大于 0.03，限制其最大值为 0.03
	
	poly
	作用：计算复音数 poly。
	解释：复音数是指合成器可以同时生成的最大音符数量。通过将 params[8] 乘以 24.9，然后加上 8，再转换为整数类型。
	
	*/
	size = (int32)(12.0f * params[2] - 6.0f);
	sizevel = 0.12f * params[3];
	muffvel = params[5] * params[5] * 5.0f;

	velsens = 1.0f + params[6] + params[6];
	if (params[6] < 0.25f) velsens -= 0.75f - 3.0f * params[6];

	fine = params[9] - 0.5f;
	random = 0.077f * params[10] * params[10];
	stretch = 0.000434f * (params[11] - 0.5f);

	cdep = params[7] * params[7];
	trim = 1.50f - 0.79f * cdep;
	width = 0.04f * params[7];  if (width > 0.03f) width = 0.03f;

	poly = 8 + (int32)(24.9f * params[8]);
}

}}} // namespaces

