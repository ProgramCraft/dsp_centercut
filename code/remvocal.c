#include "stdio.h"
#include <math.h>
#include <windows.h>
#define min(a,b)	((a)<(b)?(a):(b))
#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif
typedef unsigned char               uint8;
typedef unsigned char               bool;
typedef unsigned int                uint32;
#define TRUE                        (1==1)
#define FALSE                       (1==0)
#include "centercut.h"
bool			mInitialized = FALSE;
//HANDLE			hMutexRunning = 0;

#define trce	printf

#define	kWindowSize	4096
unsigned int bigbuf[0xa000];
unsigned *mBitRev = (bigbuf + 0x800);
float *mPreWindow = (bigbuf + 0x800 + kWindowSize);//[kWindowSize];
float	*mPostWindow = (bigbuf + 0x800 + (kWindowSize * 2));
float *mSineTab = (bigbuf + 0x800 + (kWindowSize * 3));
float *mTempLBuffer = (bigbuf + 0x800 + (kWindowSize * 4));
float *mTempRBuffer = (bigbuf + 0x800 + (kWindowSize * 5));
float *mTempCBuffer = (bigbuf + 0x800 + (kWindowSize * 6));
#define centercut_pubpool	(bigbuf+0x800+(kWindowSize*7))

#define	KOVERLAPCOUNT	4
#define	MOUTPUTMAXBUFFERS 	32
const int		kPostWindowPower = 2;  // Maximum power including pre-window is KOVERLAPCOUNT-1,
									   // which means this can be KOVERLAPCOUNT-2 at most
#define	kHalfWindow (kWindowSize>>1)
#define	kOverlapSize	(kWindowSize/KOVERLAPCOUNT)

const float	twopi = 6.283185;
const float	invsqrt2 = 0.707107;
const float	nodivbyzero = 0.000001;
const int		BYTES_TO_DOUBLE = 0;
const int		DOUBLE_TO_BYTES = 1;
const int		mOutputSampleCount = kOverlapSize;
int				mOutputReadSampleOffset;
int				mOutputBufferCount;  // How many buffers are actually in use (there may be more
									 // allocated than in use)
float			*mOutputBuffer[MOUTPUTMAXBUFFERS];
int				mSampleRate;
bool			mOutputCenter;
bool			mBassToSides;
int				mOutputDiscardBlocks;
uint32			mInputSamplesNeeded;
uint32			mInputPos;
static float			mInput_array[kWindowSize][2];
float			mOverlapC[KOVERLAPCOUNT - 1][kOverlapSize];


void Lock(bool bRunning) {
	if (bRunning) {
		//WaitForSingleObject(hMutexRunning, INFINITE);
	}
	else {
		//ReleaseMutex(hMutexRunning);
	}
}

int Init_CenterCut(void) {
	Lock(TRUE);
	OutputBufferInit();
	CenterCut_Start();
	mInitialized = TRUE;
	Lock(FALSE);
	return 0;
}

void Quit_CenterCut() {
	Lock(TRUE);
	CenterCut_Finish();
	OutputBufferFree();
	mInitialized = FALSE;
	Lock(FALSE);
}

int ModifySamples_Sides(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate) {
	Lock(TRUE);
	if ((chanCount == 2) && (sampleCount > 0) && BPSIsValid(bitsPerSample) && mInitialized) {
		int outSampleCount = CenterCutProcessSamples(samples, sampleCount, samples, bitsPerSample, sampleRate, FALSE, FALSE);

		if (outSampleCount >= 0) {
			sampleCount = outSampleCount;
		}
	}
	Lock(FALSE);

	return sampleCount;
}

int ModifySamples_Center(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate) {
	Lock(TRUE);
	if ((chanCount == 2) && (sampleCount > 0) && BPSIsValid(bitsPerSample) && mInitialized) {
		int outSampleCount = CenterCutProcessSamples(samples, sampleCount, samples, bitsPerSample, sampleRate, TRUE, FALSE);

		if (outSampleCount >= 0) {
			sampleCount = outSampleCount;
		}
	}
	Lock(FALSE);

	return sampleCount;
}

int ModifySamples_SidesBTS(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate) {
	Lock(TRUE);
	if ((chanCount == 2) && (sampleCount > 0) && BPSIsValid(bitsPerSample) && mInitialized) {
		int outSampleCount = CenterCutProcessSamples(samples, sampleCount, samples, bitsPerSample, sampleRate, FALSE, TRUE);

		if (outSampleCount >= 0) {
			sampleCount = outSampleCount;
		}
	}
	Lock(FALSE);

	return sampleCount;
}

int ModifySamples_CenterBTS(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate) {
	Lock(TRUE);
	if ((chanCount == 2) && (sampleCount > 0) && BPSIsValid(bitsPerSample) && mInitialized) {
		int outSampleCount = CenterCutProcessSamples(samples, sampleCount, samples, bitsPerSample, sampleRate, TRUE, TRUE);

		if (outSampleCount >= 0) {
			sampleCount = outSampleCount;
		}
	}
	Lock(FALSE);

	return sampleCount;
}

int ModifySamples_Classic(uint8 *samples, int sampleCount, int bitsPerSample, int chanCount, int sampleRate) {
	Lock(TRUE);
	if ((chanCount == 2) && sampleCount && BPSIsValid(bitsPerSample)) {
		// float *sampD = new float[sampleCount*chanCount];
		float *sampD = (float *)malloc(sampleCount*chanCount * sizeof(float));
		float *tmp = sampD;

		ConvertSamples(BYTES_TO_DOUBLE, samples, sampD, sampleCount, bitsPerSample, chanCount);

		for (int i = 0; i < sampleCount; i++) {
			float diff = (tmp[0] - tmp[1]) * 0.5;
			tmp[1] = tmp[0] = diff;
			tmp += 2;
		}

		ConvertSamples(DOUBLE_TO_BYTES, samples, sampD, sampleCount, bitsPerSample, chanCount);

		// delete[] sampD;
		free(sampD);
	}
	Lock(FALSE);

	return sampleCount;
}

int CenterCutProcessSamples(uint8 *inSamples, int inSampleCount, uint8 *outSamples, int bitsPerSample, int sampleRate, bool outputCenter, bool bassToSides) {
	int bytesPerSample, outSampleCount, maxOutSampleCount, copyCount;

	mSampleRate = sampleRate;
	mOutputCenter = outputCenter;
	mBassToSides = bassToSides;
	bytesPerSample = bitsPerSample / 8;
	outSampleCount = 0;
	maxOutSampleCount = inSampleCount;

	while (inSampleCount > 0) {
		copyCount = min((int)mInputSamplesNeeded, inSampleCount);
		ConvertSamples(BYTES_TO_DOUBLE, inSamples, &mInput_array[mInputPos][0], copyCount, bitsPerSample, 2);
		inSamples += copyCount * bytesPerSample * 2;
		inSampleCount -= copyCount;
		mInputPos = (mInputPos + copyCount) & (kWindowSize - 1);
		mInputSamplesNeeded -= copyCount;

		if (mInputSamplesNeeded == 0) {
			CenterCut_Run();
		}
	}

	while ((mOutputBufferCount > 0) && (outSampleCount < maxOutSampleCount)) {
		float *sampD = mOutputBuffer[0];
		if (!sampD) return -1;

		copyCount = min(mOutputSampleCount - mOutputReadSampleOffset,
			maxOutSampleCount - outSampleCount);

		ConvertSamples(DOUBLE_TO_BYTES, outSamples, sampD + (mOutputReadSampleOffset * 2), copyCount, bitsPerSample, 2);

		outSamples += copyCount * bytesPerSample * 2;
		outSampleCount += copyCount;
		mOutputReadSampleOffset += copyCount;
		if (mOutputReadSampleOffset == mOutputSampleCount) {
			OutputBufferReadComplete();
		}
	}

	return outSampleCount;
}

void ConvertSamples(int type, uint8 *sampB, float *sampD, int sampleCount, int bitsPerSample, int chanCount) {
	const float SampleScaleInv = 32768.0;
	//const float SampleScale = 1.0 / SampleScaleInv;
	const float SampleMin = -2147483648.0;
	const float SampleMax = 2147483647.0;

	int bytesPerSample, shiftCount;
	int xor;
	uint8 *max;

	bytesPerSample = (bitsPerSample + 7) / 8;
	shiftCount = (4 - bytesPerSample) * 8;
	xor = (bytesPerSample == 1) ? (1 << 31) : 0;
	max = sampB + (sampleCount * bytesPerSample * chanCount);

	if (type == BYTES_TO_DOUBLE) {
		int tempI;

		while (sampB < max) {
			tempI = (*((int*)sampB) << shiftCount) ^ xor;
			//*sampD = (float)tempI * SampleScale;
			*sampD = (float)(tempI >> 15);
			sampB += bytesPerSample;
			sampD += 1;
		}
	}
	else {
		uint8 *maxw = max - 3;
		float tempD;
		uint32 tempI;
		while (sampB < max) {
			tempD = *sampD * SampleScaleInv;
			if (tempD > 0.0) {
				if (tempD > SampleMax) {
					tempD = SampleMax;
				}
				tempD += 0.5;
			}
			else {
				if (tempD < SampleMin) {
					tempD = SampleMin;
				}
				tempD -= 0.5;
			}
			tempI = (uint32)((int)tempD ^ xor) >> shiftCount;

			if (sampB < maxw) {
				*((uint32*)sampB) = tempI;
			}
			else {
				memcpy(sampB, &tempI, bytesPerSample);
			}

			sampB += bytesPerSample;
			sampD += 1;
		}
	}
}

void OutputBufferInit() {
	for (int i = 0; i < MOUTPUTMAXBUFFERS; i++) {
		mOutputBuffer[i] = 0;
	}
	mOutputBufferCount = 0;
	mOutputReadSampleOffset = 0;
}

void OutputBufferFree() {
	for (int i = 0; i < MOUTPUTMAXBUFFERS; i++) {
		if (mOutputBuffer[i]) {
			// delete[] mOutputBuffer[i];
			free(mOutputBuffer[i]);
			mOutputBuffer[i] = 0;
		}
	}
}

void OutputBufferReadComplete() {
	mOutputBufferCount--;
	mOutputReadSampleOffset = 0;
	if (mOutputBufferCount > 0) {
		int i;
		float *moveToEnd = mOutputBuffer[0];

		// Shift the buffers so that the current one for reading is at index 0
		for (i = 1; i < MOUTPUTMAXBUFFERS; i++) {
			mOutputBuffer[i - 1] = mOutputBuffer[i];
		}
		mOutputBuffer[MOUTPUTMAXBUFFERS - 1] = 0;

		// Move the previous first buffer to the end (first null pointer)
		for (i = 0; i < MOUTPUTMAXBUFFERS; i++) {
			if (!mOutputBuffer[i]) {
				mOutputBuffer[i] = moveToEnd;
				break;
			}
		}
	}
}

bool OutputBufferBeginWrite() {
	if (mOutputBufferCount == MOUTPUTMAXBUFFERS) {
		return FALSE;
	}

	int i = mOutputBufferCount;
	if (!mOutputBuffer[i]) {
		// No buffer exists at this index, make a new one
		// mOutputBuffer[i] = new float[mOutputSampleCount * 2];
		mOutputBuffer[i] = (float *)malloc(sizeof(float)*(mOutputSampleCount * 2));
		if (!mOutputBuffer[i]) {
			return FALSE;
		}
	}

	mOutputBufferCount++;
	return TRUE;
}

bool BPSIsValid(int bitsPerSample) {
	// Bits per sample must be between 8 and 32 bits, and a multiple of 8
	return (bitsPerSample >= 8) && (bitsPerSample <= 32) && ((bitsPerSample & 7) == 0);
}

unsigned IntegerLog2(unsigned v) {
	unsigned i = 0;

	while (v>1) {
		++i;
		v >>= 1;
	}

	return i;
}

unsigned RevBits(unsigned x, unsigned bits) {
	unsigned y = 0;

	while (bits--) {
		y = (y + y) + (x & 1);
		x >>= 1;
	}

	return y;
}

void VDCreateRaisedCosineWindow(float *dst, int n, float power) {
	const float twopi_over_n = twopi / n;
	const float scalefac = 1.0 / n;

	for (int i = 0; i<n; ++i) {
		dst[i] = scalefac * pow(0.5*(1.0 - cos(twopi_over_n * (i + 0.5))), power);
	}
}

void VDCreateHalfSineTable(float *dst, int n) {
	const float twopi_over_n = twopi / n;

	for (int i = 0; i<n; ++i) {
		dst[i] = sin(twopi_over_n * i);
	}
}

void VDCreateBitRevTable(unsigned *dst, int n) {
	unsigned bits = IntegerLog2(n);

	for (int i = 0; i<n; ++i) {
		dst[i] = RevBits(i, bits);
	}
}

void CreatePostWindow(float *dst, int windowSize, int power) {
	const float powerIntegrals[8] = { 1.0, 1.0 / 2.0, 3.0 / 8.0, 5.0 / 16.0, 35.0 / 128.0,
		63.0 / 256.0, 231.0 / 1024.0, 429.0 / 2048.0 };
	const float scalefac = (float)windowSize * (powerIntegrals[1] / powerIntegrals[power + 1]);

	VDCreateRaisedCosineWindow(dst, windowSize, (float)power);

	for (int i = 0; i<windowSize; ++i) {
		dst[i] *= scalefac;
	}
}

void VDComputeFHT(float *A, int nPoints, const float *sinTab) {
	int i, n, n2, theta_inc;

	// FHT - stage 1 and 2 (2 and 4 points)

	for (i = 0; i<nPoints; i += 4) {
		const float	x0 = A[i];
		const float	x1 = A[i + 1];
		const float	x2 = A[i + 2];
		const float	x3 = A[i + 3];

		const float	y0 = x0 + x1;
		const float	y1 = x0 - x1;
		const float	y2 = x2 + x3;
		const float	y3 = x2 - x3;

		A[i] = y0 + y2;
		A[i + 2] = y0 - y2;

		A[i + 1] = y1 + y3;
		A[i + 3] = y1 - y3;
	}

	// FHT - stage 3 (8 points)

	for (i = 0; i<nPoints; i += 8) {
		float alpha, beta;

		alpha = A[i + 0];
		beta = A[i + 4];

		A[i + 0] = alpha + beta;
		A[i + 4] = alpha - beta;

		alpha = A[i + 2];
		beta = A[i + 6];

		A[i + 2] = alpha + beta;
		A[i + 6] = alpha - beta;

		alpha = A[i + 1];

		const float beta1 = invsqrt2*(A[i + 5] + A[i + 7]);
		const float beta2 = invsqrt2*(A[i + 5] - A[i + 7]);

		A[i + 1] = alpha + beta1;
		A[i + 5] = alpha - beta1;

		alpha = A[i + 3];

		A[i + 3] = alpha + beta2;
		A[i + 7] = alpha - beta2;
	}

	n = 16;
	n2 = 8;
	theta_inc = nPoints >> 4;

	while (n <= nPoints) {
		for (i = 0; i<nPoints; i += n) {
			int j;
			int theta = theta_inc;
			float alpha, beta;
			const int n4 = n2 >> 1;

			alpha = A[i];
			beta = A[i + n2];

			A[i] = alpha + beta;
			A[i + n2] = alpha - beta;

			alpha = A[i + n4];
			beta = A[i + n2 + n4];

			A[i + n4] = alpha + beta;
			A[i + n2 + n4] = alpha - beta;

			for (j = 1; j<n4; j++) {
				float	sinval = sinTab[theta];
				float	cosval = sinTab[theta + (nPoints >> 2)];

				float	alpha1 = A[i + j];
				float	alpha2 = A[i - j + n2];
				float	beta1 = A[i + j + n2] * cosval + A[i - j + n] * sinval;
				float	beta2 = A[i + j + n2] * sinval - A[i - j + n] * cosval;

				theta += theta_inc;

				A[i + j] = alpha1 + beta1;
				A[i + j + n2] = alpha1 - beta1;
				A[i - j + n2] = alpha2 + beta2;
				A[i - j + n] = alpha2 - beta2;
			}
		}

		n *= 2;
		n2 *= 2;
		theta_inc >>= 1;
	}
}

bool CenterCut_Start() {
	float *tmp = (float *)centercut_pubpool;

	//burstObviousEvent(__LINE__,0xEEEEEEEE);
	VDCreateBitRevTable(mBitRev, kWindowSize);
	VDCreateHalfSineTable(mSineTab, kWindowSize);

	mInputSamplesNeeded = kOverlapSize;
	mInputPos = 0;

	mOutputDiscardBlocks = KOVERLAPCOUNT - 1;

	memset(mInput_array, 0, sizeof mInput_array);
	memset(mOverlapC, 0, sizeof mOverlapC);

	// float *tmp = new float[kWindowSize];

	VDCreateRaisedCosineWindow(tmp, kWindowSize, 1.0);
	for (unsigned i = 0; i<kWindowSize; ++i) {
		// The correct Hartley<->FFT conversion is:
		//
		//	Fr(i) = 0.5(Hr(i) + Hi(i))
		//	Fi(i) = 0.5(Hr(i) - Hi(i))
		//
		// We omit the 0.5 in both the forward and reverse directions,
		// so we have a 0.25 to put here.

		mPreWindow[i] = tmp[mBitRev[i]] * 0.5 * (2.0 / (float)KOVERLAPCOUNT);
	}
	CreatePostWindow(mPostWindow, kWindowSize, kPostWindowPower);
	memset(bigbuf, 0, 1024);
	return TRUE;
}

void CenterCut_Finish() {
}

bool CenterCut_Run() {
	unsigned i;
	int freqBelowToSides = (int)((200.0 / ((float)mSampleRate / kWindowSize)) + 0.5);
	// copy to temporary buffer and FHT

	for (i = 0; i<kWindowSize; ++i) {
		const unsigned j = mBitRev[i];
		const unsigned k = (j + mInputPos) & (kWindowSize - 1);
		const float w = mPreWindow[i];

		mTempLBuffer[i] = mInput_array[k][0] * w;
		mTempRBuffer[i] = mInput_array[k][1] * w;
	}

	VDComputeFHT(mTempLBuffer, kWindowSize, mSineTab);
	VDComputeFHT(mTempRBuffer, kWindowSize, mSineTab);

	// perform stereo separation

	mTempCBuffer[0] = 0;
	mTempCBuffer[1] = 0;
	for (i = 1; i<kHalfWindow; i++) {
		float lR = mTempLBuffer[i] + mTempLBuffer[kWindowSize - i];
		float lI = mTempLBuffer[i] - mTempLBuffer[kWindowSize - i];
		float rR = mTempRBuffer[i] + mTempRBuffer[kWindowSize - i];
		float rI = mTempRBuffer[i] - mTempRBuffer[kWindowSize - i];

		float sumR = lR + rR;
		float sumI = lI + rI;
		float diffR = lR - rR;
		float diffI = lI - rI;

		float sumSq = sumR*sumR + sumI*sumI;
		float diffSq = diffR*diffR + diffI*diffI;
		float alpha = 0.0;

		if (sumSq > nodivbyzero) {
			alpha = 0.5 - sqrt(diffSq / sumSq) * 0.5;
		}

		float cR = sumR * alpha;
		float cI = sumI * alpha;

		if (mBassToSides && (i < freqBelowToSides)) {
			cR = cI = 0.0;
		}

		mTempCBuffer[mBitRev[i]] = cR + cI;
		mTempCBuffer[mBitRev[kWindowSize - i]] = cR - cI;
	}

	// reconstitute left/right/center channels

	VDComputeFHT(mTempCBuffer, kWindowSize, mSineTab);

	// apply post-window

	for (i = 0; i<kWindowSize; i++) {
		mTempCBuffer[i] *= mPostWindow[i];
	}

	// writeout

	if (mOutputDiscardBlocks > 0) {
		mOutputDiscardBlocks--;
	}
	else {
		int currentBlockIndex, nextBlockIndex, blockOffset;

		if (!OutputBufferBeginWrite())
		{
			return FALSE;
		}
		float *outBuffer = mOutputBuffer[mOutputBufferCount - 1];
		if (!outBuffer)
		{
			return FALSE;
		}

		for (i = 0; i<kOverlapSize; ++i) {
			float c = mOverlapC[0][i] + mTempCBuffer[i];
			float l = mInput_array[mInputPos + i][0] - c;
			float r = mInput_array[mInputPos + i][1] - c;

			if (mOutputCenter) {
				outBuffer[0] = c;
				outBuffer[1] = c;
			}
			else {
				outBuffer[0] = l;
				outBuffer[1] = r;
			}
			outBuffer += 2;

			// overlapping

			currentBlockIndex = 0;
			nextBlockIndex = 1;
			blockOffset = kOverlapSize;
			while (nextBlockIndex < KOVERLAPCOUNT - 1) {
				mOverlapC[currentBlockIndex][i] = mOverlapC[nextBlockIndex][i] +
					mTempCBuffer[blockOffset + i];

				currentBlockIndex++;
				nextBlockIndex++;
				blockOffset += kOverlapSize;
			}
			mOverlapC[currentBlockIndex][i] = mTempCBuffer[blockOffset + i];
		}
	}

	mInputSamplesNeeded = kOverlapSize;
	return TRUE;
}

static unsigned int g_samplecount = 0;
static char *g_whole_wav_data_ptr = NULL;
static char *orien_pcm_ptr = NULL;
static unsigned int g_orien_pcm_size = 0;
static char *importFileName=NULL;
#define START_ADDR	(0)//(44100*20*2*2)
#define ALG_PROCESS_SAMPLE_NUM	512

int remvocal_init(void)
{
	DWORD filesize, readsize;
	FILE *pfile = NULL;

	if(NULL == importFileName)
		return 1;
	fopen_s(&pfile, importFileName, "r+b");
	fseek(pfile, 0, SEEK_END);
	g_orien_pcm_size = filesize = ftell(pfile);
	fseek(pfile, 0, 0);
	g_whole_wav_data_ptr = (char *)malloc(filesize + 1);
	memset(g_whole_wav_data_ptr, 0, filesize + 1);
	orien_pcm_ptr = (char *)malloc(filesize + 1 - 44);
	memset(orien_pcm_ptr, 0, filesize + 1 - 44);
	//ReadFile(pfile, g_whole_wav_data_ptr, filesize, &readsize, NULL);
	readsize = fread(g_whole_wav_data_ptr, filesize, 1, pfile);
	if (readsize != 1)
	{
		printf("read error!\n");
		return 1;
	}
	mInputSamplesNeeded = 2048;
	g_samplecount = (filesize - 44) >> 2;
	memcpy(orien_pcm_ptr, (&(g_whole_wav_data_ptr[44])), (filesize - 44));
	fclose(pfile);
	return 0;
}

void remvocal_deinit(void)
{
	free(g_whole_wav_data_ptr);
	free(orien_pcm_ptr);
}

void flush_result_pcm(void)
{
	FILE *centercut_fptr = NULL;

	//FILE *centercut_fptr = fopen("output.pcm", "wb");
	unsigned int write_len;

	fopen_s(&centercut_fptr, "output.pcm", "wb");
	if (0 != START_ADDR)
		memcpy(orien_pcm_ptr, (orien_pcm_ptr + START_ADDR), g_orien_pcm_size - 44 - START_ADDR);
	write_len = fwrite(orien_pcm_ptr, 1, (g_samplecount << 2), centercut_fptr);
	fclose(centercut_fptr);
}

int centercut_pcm_process(char *pcm_buf, int samples_num)
{
	int outSampleCount = 0;
	int processed_samples = 0, will_process_num = 0;

	while (processed_samples < samples_num)
	{
		char *buff_ptr = (pcm_buf + processed_samples * 2 * 2);
		if (processed_samples >(samples_num - ALG_PROCESS_SAMPLE_NUM))
			will_process_num = (samples_num - processed_samples);
		else
		{
			if (samples_num > ALG_PROCESS_SAMPLE_NUM)
				will_process_num = ALG_PROCESS_SAMPLE_NUM;
			else
				will_process_num = samples_num;
		}
		outSampleCount = CenterCutProcessSamples((uint8 *)buff_ptr, will_process_num, (uint8 *)buff_ptr, 16, 44100, FALSE, TRUE);
		if (outSampleCount < 0)
			return outSampleCount;
		processed_samples += will_process_num;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	int ret;
	
	if(argc < 2){
		trce("Usage :\n  ./remvocal wav_file_name\n");
		return 0;
	}
	importFileName = argv[1];
	if (strstr(importFileName, ".wav") == NULL && strstr(importFileName, ".WAV") == NULL) {
        printf("Error: import file seems not to be a wav file \n");
        return 0;
    }
	Init_CenterCut();
	ret = remvocal_init();
	if(!!ret)
		return ret;
	centercut_pcm_process(orien_pcm_ptr, g_samplecount);
	flush_result_pcm();
	remvocal_deinit();
	Quit_CenterCut();
	trce("done!\n");
    return 0;
}

