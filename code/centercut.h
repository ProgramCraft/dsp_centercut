#ifndef __CENTERCUT_H__
#define __CENTERCUT_H__
void Lock(bool bRunning);
int Init_CenterCut(void);
void Quit_CenterCut();
int ModifySamples_Sides(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate);
int ModifySamples_Center(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate);
int ModifySamples_SidesBTS(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate);
int ModifySamples_CenterBTS(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate);
int ModifySamples_Classic(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate);
int CenterCutProcessSamples(uint8 *inSamples, int inSampleCount, uint8 *outSamples, int bitsPerSample, int sampleRate, bool outputCenter, bool bassToSides);
void ConvertSamples(int type, uint8 *sampB, float *sampD, int sampleCount, int bitsPerSample, int chanCount);
void OutputBufferInit();
void OutputBufferFree();
void OutputBufferReadComplete();
bool OutputBufferBeginWrite();
bool BPSIsValid(int bitsPerSample);
bool CenterCut_Start();
void CenterCut_Finish();
bool CenterCut_Run();
int centercut_pcm_process(char *pcm_buf, int samples_num);
#endif
