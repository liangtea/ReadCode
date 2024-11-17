import wave
import numpy as np
import math
import matplotlib.pyplot as plt

kgrp = [
    {"root": 36, "high": 37, "pos": 0, "end": 36275, "loop": 14774},
    {"root": 40, "high": 41, "pos": 36278, "end": 83135, "loop": 16268},
    {"root": 43, "high": 45, "pos": 83137, "end": 146756, "loop": 33541},
    {"root": 48, "high": 49, "pos": 146758, "end": 204997, "loop": 21156},
    {"root": 52, "high": 53, "pos": 204999, "end": 244908, "loop": 17191},
    {"root": 55, "high": 57, "pos": 244910, "end": 290978, "loop": 23286},
    {"root": 60, "high": 61, "pos": 290980, "end": 342948, "loop": 18002},
    {"root": 64, "high": 65, "pos": 342950, "end": 391750, "loop": 19746},
    {"root": 67, "high": 69, "pos": 391752, "end": 436915, "loop": 22253},
    {"root": 72, "high": 73, "pos": 436917, "end": 468807, "loop": 8852},
    {"root": 76, "high": 77, "pos": 468809, "end": 492772, "loop": 9693},
    {"root": 79, "high": 81, "pos": 492774, "end": 532293, "loop": 10596},
    {"root": 84, "high": 85, "pos": 532295, "end": 560192, "loop": 6011},
    {"root": 88, "high": 89, "pos": 560194, "end": 574121, "loop": 3414},
    {"root": 93, "high": 999, "pos": 574123, "end": 586343, "loop": 2399},
]

# 读取pdata.txt文件中的PCM数据
with open('pdata.txt', 'r') as file:
    pcm_data = file.read()

# 将逗号隔开的字符串转换为列表并转换为整数类型
pcm_data = list(map(int, pcm_data.split(',')))

# 设置WAV文件的参数
sample_rate = 44100  # 采样率
num_channels = 2  # 双声道
sampwidth = 2  # 16位PCM格式

# 将数据转换为numpy数组并规范化
waves = np.array(pcm_data, dtype=np.int16)

# 按下了哪个音符！
note = 60

# 初始化
Fs = 44100.0
iFs = 1.0 / Fs
cmax = 0x7F
volume = 0.2
muff = 160.0
cpos = 0
comb = np.zeros(256, dtype=float)

# 初始化计算参数
size = int(12.0 * 0.500 - 6.0)
sizevel = 0.12 * 0.5
muffvel = 0.251 * 0.251 * 5.0
velsens = 1.0 + 0.376 + 0.376  # if (0.376f/*params[6]*/ < 0.25f) velsens -= 0.75f - 3.0f * 0.376f/*params[6]*/;
fine = 0.500 - 0.5
random = 0.077 * 0.246 * 0.246
stretch = 0.000434 * (0.500 - 0.5)
cdep = 0.500 * 0.500
trim = 1.50 - 0.79
width = 0.04 * 0.500  # if (width > 0.03f) width = 0.03f;
poly = 8 + int(24.9 * 0.330)

# 初始化音符的参数
note_velocity = 0.785714269
velocity = note_velocity * 127
k = (note - 60) * (note - 60);
l0 = fine + random * (k % 13 - 6.5)  # random & fine tune
if note > 60:
    l0 += stretch * k  # stretch

s = size
if velocity > 40:
    s += int(sizevel * (velocity - 40))

k = 0
while note > (kgrp[k]['high'] + s):
    k += 1  # find keygroup

l0 += float(note - kgrp[k]['root'])  # pitch
l0 = 22050.0 * iFs * float(math.exp(0.05776226505 * l0))
vl_delta = int(65536.0 * l0)
vl_frac = 0
vl_pos = kgrp[k]['pos']
vl_end = kgrp[k]['end']
vl_loop = kgrp[k]['loop']

vl_env = (0.5 + velsens) * float(math.pow(0.0078 * velocity, velsens))  # velocity

l0 = 50.0 + 0.803 * 0.803 * muff + muffvel * float(velocity - 64)  # muffle
if l0 < (55.0 + 0.25 * float(note)):
    l0 = 55.0 + 0.25 * float(note)
if l0 > 210.0:
    l0 = 210.0
vl_ff = l0 * l0 * iFs
vl_f0 = vl_f1 = 0.0

vl_note = note  # note->pan
if note < 12:
    note = 12
if note > 108:
    note = 108
l0 = volume * trim
vl_outr = l0 + l0 * width * float(note - 60)
vl_outl = l0 + l0 - vl_outr

if note < 44:
    note = 44  # limit max decay length
l0 = 2.0 * 0.500
if l0 < 1.0:
    l0 += 0.25 - 0.5 * 0.500

vl_dec = float(math.exp(-iFs * math.exp(-0.6 + 0.033 * float(note) - l0)))
vl_noteID = 9999

out0_list = []
out1_list = []

# 增加采样点的数量
num_samples = 44100

wave_raw = []
wave_out0 = []
wave_out1 = []
p_start = 200
p_end = 300
p_pos = 0

for _ in range(num_samples):
    l0 = r = 0.0
    vl_frac += vl_delta
    vl_pos += vl_frac >> 16
    vl_frac &= 0xFFFF
    if vl_pos > vl_end:
        vl_pos -= vl_loop
    i = waves[vl_pos] + ((vl_frac * (waves[vl_pos + 1] - waves[vl_pos])) >> 16)
    x = vl_env * float(i) / 32768.0

    vl_env = vl_env * vl_dec  # envelope
    vl_f0 += vl_ff * (x + vl_f1 - vl_f0)  # muffle filter
    vl_f1 = x

    l0 += vl_outl * vl_f0
    r += vl_outr * vl_f0

    if not (l0 > -2.0) or not (l0 < 2.0):
        l0 = 0.0
    if not (r > -2.0) or not (r < 2.0):
        r = 0.0

    comb[cpos] = l0 + r
    cpos = (cpos + 1) & cmax
    x = cdep * comb[cpos]  # stereo simulator

    out0 = l0 + x
    out1 = r - x

    out0_list.append(out0)
    out1_list.append(out1)

    if p_start < p_pos < p_end:
        wave_raw.append(waves[vl_pos])
        wave_out0.append(out0)
        wave_out1.append(out1)
    p_pos += 1

    # print("out0=%f" % out0)
    # print("out1=%f" % out1)

# 规范化输出音频数据
out0_list = np.array(out0_list)
out1_list = np.array(out1_list)
max_val = max(np.max(np.abs(out0_list)), np.max(np.abs(out1_list)))
out0_list = (out0_list / max_val * 32767).astype(np.int16)
out1_list = (out1_list / max_val * 32767).astype(np.int16)

# 创建WAV文件
wav_data = np.vstack((out0_list, out1_list)).T.flatten()

with wave.open('output.wav', 'w') as wav_file:
    wav_file.setnchannels(num_channels)
    wav_file.setsampwidth(sampwidth)
    wav_file.setframerate(sample_rate)
    wav_file.writeframes(wav_data.tobytes())

print("WAV文件已创建：output.wav")

# 生成1到100的横坐标
x = list(range(1, 100))

plt.figure(figsize=(14, 8))

# 原始波形图
plt.subplot(3, 1, 1)
plt.plot(x, wave_raw, label='Wave Raw')
plt.title('Wave Raw')
plt.xlabel('Sample Number')
plt.ylabel('Amplitude')
plt.legend()

# 左声道波形图
plt.subplot(3, 1, 2)
plt.plot(x, wave_out0, label='Wave Out0', color='r')
plt.title('Wave Out0')
plt.xlabel('Sample Number')
plt.ylabel('Amplitude')
plt.legend()

# 右声道波形图
plt.subplot(3, 1, 3)
plt.plot(x, wave_out1, label='Wave Out1', color='g')
plt.title('Wave Out1')
plt.xlabel('Sample Number')
plt.ylabel('Amplitude')
plt.legend()

plt.tight_layout()
plt.show()
