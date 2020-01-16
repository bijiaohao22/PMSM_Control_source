/*
 * HPF.cpp
 *
 *  Created on: 2020/01/16
 *      Author: watashi
 */

#include "HPF.hpp"

HPF::HPF() {
	// TODO Auto-generated constructor stub

}

HPF::~HPF() {
	// TODO Auto-generated destructor stub
}

void HPF::Init(float pGainB0, float pGainB1, float pGainA1) {
	mGainB0 = pGainB0;
	mGainB1 = pGainB1;
	mGainA1 = pGainA1;
}

float HPF::Output(float pTime, float pInput) {
	mBuf1 = (mGainB0 * pInput) - (mGainB1 * mInputUnitDelay.PushAndGetVal(pInput));
	mOutput = mBuf1 - mGainA1 * mBuf2;
	mBuf2 = mOutputUnitDelay.PushAndGetVal(mOutput);
	return mOutput;
	//TODO ここの答え合わせ
}