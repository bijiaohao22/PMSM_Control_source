/*
 * MotorCtrl.cpp
 *
 *  Created on: Aug 5, 2019
 *      Author: watashi
 */

#include "MotorCtrl.hpp"

MotorCtrl::MotorCtrl() {
	// TODO Auto-generated constructor stub

}

MotorCtrl::~MotorCtrl() {
	// TODO Auto-generated destructor stub
	DeInitSystem();
}

void MotorCtrl::InitSystem(void) {
	//以下CubeMXに頼らない定義たち
	//mainで既に定義されているとうまく動かないから、Mainで定義する前に呼び出すこと。
//	GPIOInit::Init();//これめんどい
//	USARTInit::Init();//これめんどい


	//Timer Initialize
	TIMCtrl::MX_TIM1_Init();

	TIMCtrl::MotorDuty_ch1(0);//50%duty
	TIMCtrl::MotorDuty_ch2(0);
	TIMCtrl::MotorDuty_ch3(0);
	TIMCtrl::MotorDuty_ch4(0.99);//9割9分でタイミングで打つ
	//TIMCtrl::TIM1SetCOMP_ch4(PWM_PERIOD_COUNT - 1);

	//ENABLE信号 PWMSet_Pin|OCSet_Pin|GateEnable_Pin
	HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_RESET);//6PWM
	//HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_SET);//3PWM
	HAL_GPIO_WritePin(OCSet_GPIO_Port, OCSet_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_SET);

	TIMCtrl::TIM1PWMStart();

	//Encoder Initialize
	//EncoderABZCtrl::MX_TIM4_Init();
	//EncoderABZCtrl::EncoderStart();

	mWaveGen.InitFrequency(HF_CONV_FREQ);

	//ADC Initialize
	ADCCtrl::ADC2Init_HAL();
	ADCCtrl::ADC2Calibration();

	ADCCtrl::ADC3Init_HAL();
	ADCCtrl::ADC3Calibration();
	ADCCtrl::ADC3InjectedStart_IT();
}


void MotorCtrl::DeInitSystem(void) {
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_RESET);//Gate Disable

	//以降Initの逆順にDeInitしていくこと
	ADCCtrl::ADC3DeInit_HAL();
	ADCCtrl::ADC2DeInit_HAL();

	//TIMをDeIntする前に安全のため、50%Duty,I=0に戻す
	TIMCtrl::MotorDuty_ch1(0);//50%duty
	TIMCtrl::MotorDuty_ch2(0);
	TIMCtrl::MotorDuty_ch3(0);

	//Timer Initialize
	TIMCtrl::MX_TIM1_DeInit();

	//ENABLE信号 PWMSet_Pin|OCSet_Pin|GateEnable_Pin
	HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_RESET);//6PWM
	//HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_SET);//3PWM
	HAL_GPIO_WritePin(OCSet_GPIO_Port, OCSet_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_RESET);
}


void MotorCtrl::InitMotorControl(void) {

	mControlMode = OpenLoop;//起動時、動作をまずは強制転流にする。

	if(SINEWAVE_CONVOLUTION) { //高周波重畳アプリケーション
		mControlMode = FOC_Convolution;
	}

	if(SQWAVE_CONVOLUTION) {
		mControlMode = FOC_SqWaveConvolution;
	}

	mArgCtrl.Init();

	mIganmaPID.SetParam(PID_GAIN_IGANMA_P, PID_GAIN_IGANMA_I, PID_GAIN_IGANMA_D);
	mIdeltaPID.SetParam(PID_GAIN_IDELTA_P, PID_GAIN_IDELTA_I, PID_GAIN_IDELTA_D);
	mIganmaPID.SetSampleTime(PID_CYCLE_TIME);
	mIdeltaPID.SetSampleTime(PID_CYCLE_TIME);

	mVelocityPID.SetParam(PID_GAIN_VEL_P, PID_GAIN_VEL_I, PID_GAIN_VEL_D);
	mVelocityPID.SetSampleTime(PID_CYCLE_TIME_VEL);
}

void MotorCtrl::InitObserver(void) {
	//オブザーバ
	mObserver.InitEMFObs(OBSERVER_CYCLE_TIME, M_PARAM_R, M_PARAM_LD, M_PARAM_LQ, OBSERVER_GAIN_ALPHA);
	mObserver.InitPhaseEstimator(OBSERVER_CYCLE_TIME, OBSERVER_GAIN_K1, OBSERVER_GAIN_K2, OBSERVER_GAIN_K3);

	//正弦波重畳
	mSineWaveConvCalculator.InitPhaseEstimator(OBSERVER_CYCLE_TIME, HF_PII_GAIN_K1, HF_PII_GAIN_K2, HF_PII_GAIN_K3);
	mSineWaveConvCalculator.BPF_LPFInit(HF_LPF_BPF_B0, HF_LPF_BPF_B1, HF_LPF_BPF_A1);
	mSineWaveConvCalculator.BPF_HPFInit(HF_HPF_BPF_B0, HF_HPF_BPF_B1, HF_HPF_BPF_A1);
	mSineWaveConvCalculator.LPFInit(HF_LPF_B0, HF_LPF_B1, HF_LPF_A1);

	//矩形波重畳
	mSqWaveConvCalculator.InitPhaseEstimator(OBSERVER_CYCLE_TIME, HF_PII_GAIN_K1, HF_PII_GAIN_K2, HF_PII_GAIN_K3);
	mSqWaveConvCalculator.HPFInit(SQWAVE_DEMOD_HPF_B0, SQWAVE_DEMOD_HPF_B1, SQWAVE_DEMOD_HPF_A1);

}


void MotorCtrl::HighFreqTask(void) {
	MotorDrive();
}


void MotorCtrl::MotorDrive(void) { //モータを動かすモード.他に測定モードを用意する予定

	GPIODebugTask();//GPIOからオシロに波形を出力する

	SetCurrentTarget();
	if(mControlMode == FOC_Convolution) {
		SetVhTask();
		WaveGenTask();
	} else if(mControlMode == FOC_SqWaveConvolution) {
		SetVhTaskForSqWave();
		SqWaveGenTask();
	}


	GPIODebugTask();//GPIOからオシロに波形を出力する

	ReadCurrentTask();
	ReadVoltageTask();

	ReadAngleTask(); //mgdArgを取得する。(OpenLoop or FOC分岐あり)

	//Iuvw -> Iab
	clarkTransform();
	//Iab -> Igd
	parkabtogd();//mgdArgで回転

	GPIODebugTask();//GPIOからオシロに波形を出力する

	ObserverTask();//オブザーバ
	VelocityPIDTask();//速度PID制御

	CurrentControlTask();//電流制御とか

	GPIODebugTask();//GPIOからオシロに波形を出力する

	//Vgd　->　Vab
	invParkgdtoab();
	//Vab -> Vuvw
	SVM();

	VoltageOutputTask();//PWM出力

	ControlModeHandler();//次の運転制御モードを決める関数

	//DEBUG
	GPIODebugTask();//GPIOからオシロに波形を出力する

	if(mDEBUGorSPI == SPI) {
		SPITask();//エラッタなどの理由でとても重いから、２回のうち１回にする
		//mDebugCtrl.RTTOutput(mMotorInfo, mUIStatus);
		mDebugCtrl.AddOutputString(mMotorInfo);

	}
	if(mDEBUGorSPI == Debug) {
		if(DEBUG_MODE){//デバッグモードで入る処理
			mDebugCtrl.RTTOutput(mMotorInfo, mUIStatus);
		}
	}

	GPIODebugTask();//GPIOからオシロに波形を出力する
}


void MotorCtrl::SPITask(void) {
	SPICtrl::GetInstance().SPITransmitReceive();
}


void MotorCtrl::SetCurrentTarget() {
	std::array<int,(SPI_DATA_SIZE/4)> rxint = SPICtrl::GetInstance().GetRxInt();
	mMotorInfo.mCurrentTargetInput = (float)rxint.at(0)/(float)4095;
}

void MotorCtrl::SetVhTask() {
	std::array<int,(SPI_DATA_SIZE/4)> rxint = SPICtrl::GetInstance().GetRxInt();
	float Vh = (float)rxint.at(1)/(float)4095;

	if(HF_ARG_ZERO_FIX) {
		mMotorInfo.mVh = Vh;
	} else {
		mMotorInfo.mVh = 0.3; //通常時は0.3固定にする
	}

	//高周波重畳・取得Δθの倍率調整
	if( mControlMode == FOC_Convolution ){
		constexpr float VhGain = ( HF_CONV_FREQ * M_PARAM_LD * M_PARAM_LQ /(M_PARAM_LD - M_PARAM_LQ) / 2.0f );
		mSineWaveConvCalculator.SetKh(VhGain/mMotorInfo.mVh);
	} else if( mControlMode == FOC_SqWaveConvolution ) {
		constexpr float VhGain = ( 2 * CONV_SQWAVE_FREQ * M_PARAM_LD * M_PARAM_LQ /(M_PARAM_LQ - M_PARAM_LD) / M_PI );
		mSqWaveConvCalculator.SetKh(VhGain/mMotorInfo.mVh);
	}

}


void MotorCtrl::SetVhTaskForSqWave() {
	std::array<int,(SPI_DATA_SIZE/4)> rxint = SPICtrl::GetInstance().GetRxInt();
	float Vh = (float)rxint.at(1)/(float)4095;

	if(HF_ARG_ZERO_FIX) {
		mMotorInfo.mVh = Vh;
	} else {
		mMotorInfo.mVh = 0.3; //通常時は0.3固定にする
	}
}

void MotorCtrl::WaveGenTask() {
	std::array<float,4> waves = mWaveGen.OutputWavesSupOffsetPhase_dq(HF_HETERODYNE_PHASE_OFFSET_D, HF_HETERODYNE_PHASE_OFFSET_Q);
	mMotorInfo.mSinForConv = waves.at(0);
	mMotorInfo.mCosForConv = waves.at(1);
	mMotorInfo.mSinForDemodulation = waves.at(2);
	mMotorInfo.mCosForDemodulation = waves.at(3);
}

void MotorCtrl::SqWaveGenTask() {
	mMotorInfo.mSqWaveForConv = mSqWaveGen.OutputSqWave();
}

void MotorCtrl::ReadCurrentTask(void) {
	//ReadCurrent
	//エンコーダ読み取り
	float Iu,Iv,Iw;
	//増幅率のバイアス考慮してない。あとで計算すること。
	Iu = (float)ADCCtrl::ADC3_INJ_Read_ch1() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	Iv = (float)ADCCtrl::ADC3_INJ_Read_ch2() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	Iw = (float)ADCCtrl::ADC3_INJ_Read_ch3() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	Iu = -Iu;//モータ「に」流す電流にするため、反転。
	Iv = -Iv;
	Iw = -Iw;
	setIuvw(Iu, Iv, Iw);
}


void MotorCtrl::ReadVoltageTask() {
	//ReadVoltage
	//電圧測定(入力)
	mMotorInfo.mVoltageVCC = VCC_VOLTAGE;
}


//Motor
void MotorCtrl::setIuvw(float pIu, float pIv, float pIw){
	mMotorInfo.mIuvw.at(0) = pIu;
	mMotorInfo.mIuvw.at(1) = pIv;
	mMotorInfo.mIuvw.at(2) = pIw;
}

void MotorCtrl::ReadAngleTask(void) {
	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {
		mMotorInfo.mgdArg = GetAngleForOpenLoop();
	}else if(mControlMode == FOC) {
		mMotorInfo.mgdArg = GetAngleForFOC();
	}else if(mControlMode == FOC_Convolution) {
		mMotorInfo.mgdArg = GetAngleForSineWaveConv();
	}else if(mControlMode == FOC_SqWaveConvolution) {
		mMotorInfo.mgdArg = GetAngleForSqWaveConv();
	}
}


fp_rad MotorCtrl::GetAngleForOpenLoop(void) {
	//強制転流では、gd軸を回転させる。
	if(mUIStatus.mStartStopTRG) {
	mArgCtrl.accelerationForOpenLoop();
	} else {
	mArgCtrl.decelerationForOpenLoop();
	}
	return mArgCtrl.getArg(); //gd軸のみ回す。
}


fp_rad MotorCtrl::GetAngleForFOC(void) {
	return mObserver.GetEstTheta();//オブザーバから読み取る
}


fp_rad MotorCtrl::GetAngleForSineWaveConv(void) {
	//ここで調整　発散なら0
	if(HF_ARG_ZERO_FIX) {
		return 0;
	}
	return mSineWaveConvCalculator.GetTheta_c();
}


fp_rad MotorCtrl::GetAngleForSqWaveConv(void) {
	//ここで調整　発散なら0
	if(HF_ARG_ZERO_FIX) {
		return 0;
	}
	return mSqWaveConvCalculator.GetTheta_c();
}



void MotorCtrl::clarkTransform(void) {
	std::array<float, 3> Iuvw = {mMotorInfo.mIuvw.at(0),
											   mMotorInfo.mIuvw.at(1),
											   -mMotorInfo.mIuvw.at(0) - mMotorInfo.mIuvw.at(1)};
	std::array<float, 2> Iab = MotorMath::clarkTransform(Iuvw);
	mMotorInfo.mIab = Iab;
}

void MotorCtrl::parkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Idq = MotorMath::parkTransform(dqArg, Iab);
	mMotorInfo.mIdq = Idq;
}

void MotorCtrl::parkGanmaDelta(void) {
	fp_rad ArgErr = mMotorInfo.mArgErr;
	std::array<float, 2> Idq = mMotorInfo.mIdq;
	fp_rad InvArgErr = -1.0f * ArgErr;
	std::array<float, 2> Igd = MotorMath::parkTransform(InvArgErr, Idq);
	mMotorInfo.mIgd = Igd;
}

//optional function
void MotorCtrl::parkabtogd(void) {
	fp_rad gdArg = mMotorInfo.mgdArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Igd = MotorMath::parkTransform(gdArg, Iab);
	mMotorInfo.mIgd = Igd;
}

std::array<float, 2> MotorCtrl::getIdq() {
	return mMotorInfo.mIdq;
}

std::array<float, 2> MotorCtrl::getIgd() {
	return mMotorInfo.mIgd;
}


void MotorCtrl::ObserverTask() {
	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.CalculateOpenLoop( mArgCtrl.getArgOmega() ,mMotorInfo.mgdArg );//強制転流中はこっち。

		//MotorInfoへ情報の格納
		mMotorInfo.mArgErr = mObserver.GetEstAxiErr(); //軸誤差。gdとdqの差。本来はこの情報だけでドライブできる。
		mMotorInfo.mEstOmega = mObserver.GetEstOmegaE();//デバッグ用
		mMotorInfo.mEstTheta = mObserver.GetEstTheta();//デバッグ用

	}else if(mControlMode == FOC) {
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.Calculate();//ベクトル制御用

		//MotorInfoへ情報の格納
		mMotorInfo.mArgErr = mObserver.GetEstAxiErr(); //軸誤差。gdとdqの差。本来はこの情報だけでドライブできる。
		mMotorInfo.mEstOmega = mObserver.GetEstOmegaE();//デバッグ用
		mMotorInfo.mEstTheta = mObserver.GetEstTheta();//デバッグ用

	}else if(mControlMode == FOC_Convolution) {
		//高周波重畳位置推定
		mSineWaveConvCalculator.SetIgdPair(mMotorInfo.mIgd);
		mSineWaveConvCalculator.SetSinCosForDemodulation( {mMotorInfo.mSinForDemodulation,mMotorInfo.mCosForDemodulation} );
		mSineWaveConvCalculator.AngleCalculate();

		//オブザーバ切り替え推定電気角速度
		mMotorInfo.mEstOmega_HF = mSineWaveConvCalculator.GetEstOmegaE();

		//オブザーバ位置推定
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.Calculate();//ベクトル制御用

		//高周波重畳推定電気角速度
		mMotorInfo.mEstOmega_Observer = mObserver.GetEstOmegaE();

		//高周波重畳,Rフィルタ,位相遅れ制御,設計用Debug
		mMotorInfo.mConvIdqc = mSineWaveConvCalculator.GetConvIdqc();
		mMotorInfo.mIdqch = mSineWaveConvCalculator.GetIdqch();

		//出力のThetaE
		mMotorInfo.mEstTheta = mSineWaveConvCalculator.GetTheta_c();
	}else if(mControlMode == FOC_SqWaveConvolution) {
		mSqWaveConvCalculator.SetIgdPair(mMotorInfo.mIgd);
		mSqWaveConvCalculator.SetSqWave(mMotorInfo.mSqWaveForConv);

		mSqWaveConvCalculator.AngleCalculate();//Demodulation

		mMotorInfo.mEstOmega_HF = mSqWaveConvCalculator.GetEstOmegaE();
	}

}


void MotorCtrl::VelocityPIDTask() {
	if(mControlMode == FOC) {
			float velocityTarget = 1500;
			float velErr = velocityTarget - mMotorInfo.mEstOmega;
			mVelocityPID.ErrorUpdate(velErr);
	}
}

void MotorCtrl::CurrentControlTask() {
	//Current Target Setting
	mMotorInfo.mIgdTarget = GetCurrentTarget();

	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {

		CurrentFeedForwardTask();//OpenLoop Drive

	} else if (mControlMode == FOC) {
		CurrentPITask();//PI Control

	} else if (mControlMode == FOC_Convolution || mControlMode == FOC_SqWaveConvolution) {
		CurrentPITaskForConvolution();//PI Control

	}

}

std::array<float, 2> MotorCtrl::GetCurrentTarget() {
	//mMotorInfo.mIgdTargetを操作するTask

	std::array<float, 2> IgdTarget = {0, 0};
	float CurrentTargetInput = mMotorInfo.mCurrentTargetInput;

	if(mControlMode == OpenLoop) {

		IgdTarget.at(0) = CurrentTargetInput;//IgTarget [A]
		IgdTarget.at(1) = 0;//IdTarget [A]

		return IgdTarget;

	}else if(mControlMode == OpenLoopToFOC) {//OpenLoopからFOCに切り替わる時に動作するモード

		if(mTransitionCountForOpenToFOC < OPEN_TO_FOC_TRANSITION_COUNT_STEP1) {
			IgdTarget.at(0) = CurrentTargetInput * (OPEN_TO_FOC_TRANSITION_COUNT_STEP1 - mTransitionCountForOpenToFOC) / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			IgdTarget.at(1) = CurrentTargetInput * mTransitionCountForOpenToFOC / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mTransitionCountForOpenToFOC++;
			return IgdTarget;
		} else {
			IgdTarget.at(0) = CurrentTargetInput * (OPEN_TO_FOC_TRANSITION_COUNT_STEP1 - mTransitionCountForOpenToFOC) / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			IgdTarget.at(1) = CurrentTargetInput * mTransitionCountForOpenToFOC / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mTransitionCountForOpenToFOC2++;
			return IgdTarget;
		}

	}else if(mControlMode == FOC) {//FOCのときの入力
		IgdTarget.at(0) = 0;
		IgdTarget.at(1) = CurrentTargetInput/5;


		if(VEL_CONTROL) { //ここおかしい。これはHanlderが持つべき仕事であって、電流指令決定が持つべきものではない
			if(mFOCcount>30000){
				IgdTarget.at(1) = mVelocityPID.OutPut();
			} else {
			mFOCcount++;
			}
		}

		return IgdTarget;

	}else if(mControlMode == FOC_Convolution) {
//		IgdTarget.at(0) = CurrentTargetInput;//Idcに指令電流いれてみる。
//		IgdTarget.at(1) = 0;//IqcTarget [A]

		if(HF_ARG_ZERO_FIX) {
			IgdTarget.at(0) = CurrentTargetInput*4;//Idcに指令電流いれてみる。
			IgdTarget.at(1) = 0;//IqcTarget [A]
			return IgdTarget;
		}

		IgdTarget.at(0) = 0;//Idcに指令電流いれてみる。
		IgdTarget.at(1) = CurrentTargetInput*4;//IqcTarget [A]


		return IgdTarget;
	}

}

void MotorCtrl::CurrentFeedForwardTask() {
	//作成途中 OpenLoop用のFeedForward制御をする。

	//モータの電圧方程式を参考に、出力電流から出力電圧を計算する
	float IgTarget = mMotorInfo.mIgdTarget.at(0);
	float IdTarget = mMotorInfo.mIgdTarget.at(1);

	float R = M_PARAM_R;
	float Ld = M_PARAM_LD;
	float Lq = M_PARAM_LQ;
	float Ke = M_PARAM_PHY;

	float omega = mArgCtrl.getArgOmega();

	float Vganma = R * IgTarget - omega * Lq * IdTarget;
	float Vdelta = R * IdTarget - omega * Ld * IgTarget + Ke * omega;

	setVgd({Vganma, Vdelta});
}

void MotorCtrl::CurrentPITask() {
	//PI Control Start
	mMotorInfo.mIgdErr.at(0) = mMotorInfo.mIgdTarget.at(0) - mMotorInfo.mIgd.at(0);
	mMotorInfo.mIgdErr.at(1) = mMotorInfo.mIgdTarget.at(1) - mMotorInfo.mIgd.at(1);
	std::array<float,2> CulcVgd = PIDgd_control(mMotorInfo.mIgdErr);

	setVgd(CulcVgd);
}


void MotorCtrl::CurrentPITaskForConvolution() {

	//PI Control Start
	mMotorInfo.mIgdErr.at(0) = mMotorInfo.mIgdTarget.at(0) - mMotorInfo.mIgd.at(0);
	mMotorInfo.mIgdErr.at(1) = mMotorInfo.mIgdTarget.at(1) - mMotorInfo.mIgd.at(1);
	std::array<float,2> CulcVgd = PIDgd_control(mMotorInfo.mIgdErr);

	//ここで重畳させる
	float VgConv,VdConv;
	if(mControlMode == FOC_Convolution) {
		VgConv = mMotorInfo.mVh * mMotorInfo.mCosForConv;
		VdConv = mMotorInfo.mVh * mMotorInfo.mSinForConv;
	} else if (mControlMode == FOC_SqWaveConvolution) {
		VgConv = mMotorInfo.mVh * mMotorInfo.mSqWaveForConv;
		VdConv = 0;
	}

	CulcVgd.at(0) = CulcVgd.at(0) + VgConv;
	CulcVgd.at(1) = CulcVgd.at(1) + VdConv;

	setVgd(CulcVgd);
}


std::array<float, 2> MotorCtrl::PIDgd_control(const std::array<float, 2> &pErrIgd) {
		float ErrIganma = pErrIgd.at(0);
		float ErrIdelta = pErrIgd.at(1);
		mIganmaPID.ErrorUpdate(ErrIganma);
		mIdeltaPID.ErrorUpdate(ErrIdelta);

		float Vganma = mMotorInfo.mVgd.at(0);
		float Vdelta = mMotorInfo.mVgd.at(1);

		Vganma = mIganmaPID.OutPut();
		Vdelta = mIdeltaPID.OutPut();

		return {Vganma, Vdelta};
}




void MotorCtrl::setVgd(std::array<float, 2> pVgd) {
	float Vganma = pVgd.at(0);
	float Vdelta = pVgd.at(1);
	//上限Limit
	if( Vganma > PID_IGANMA_MAX_VOLTAGE ) {
		Vganma = PID_IGANMA_MAX_VOLTAGE;
	}
	if( Vdelta > PID_IDELTA_MAX_VOLTAGE ) {
		Vdelta = PID_IDELTA_MAX_VOLTAGE;
	}
	//下限Limit
	if( Vganma < PID_IGANMA_MIN_VOLTAGE ) {
		Vganma = PID_IGANMA_MIN_VOLTAGE;
	}
	if( Vdelta < PID_IDELTA_MIN_VOLTAGE ) {
		Vdelta = PID_IDELTA_MIN_VOLTAGE;
	}
	//Limitを適用
	pVgd.at(0) = Vganma;
	pVgd.at(1) = Vdelta;
	mMotorInfo.mVgd = pVgd;
}


//optional function
void MotorCtrl::invParkgdtoab(void) {
	std::array<float, 2> Vgd = mMotorInfo.mVgd;
	fp_rad gdArg = mMotorInfo.mgdArg;
	std::array<float, 2> Vab = MotorMath::InvparkTransform(gdArg, Vgd);
	mMotorInfo.mVab = Vab;
}


void MotorCtrl::invParkGanmaDelta(void) {
	std::array<float, 2> Vgd = mMotorInfo.mVgd;
	fp_rad ArgErr = mMotorInfo.mArgErr;
	fp_rad InvArgErr = -1.0f * ArgErr;
	std::array<float, 2> Vdq = MotorMath::InvparkTransform(InvArgErr, Vgd);
	mMotorInfo.mVdq = Vdq;
}


void MotorCtrl::invParkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Vdq= mMotorInfo.mVdq;
	std::array<float, 2> Vab = MotorMath::InvparkTransform(dqArg, Vdq);
	mMotorInfo.mVab = Vab;
}


void MotorCtrl::invClarkTransform(void) {
	std::array<float, 2> Vab = {mMotorInfo.mVab.at(0)/mMotorInfo.mVoltageVCC,
											  mMotorInfo.mVab.at(1)/mMotorInfo.mVoltageVCC};
	std::array<float, 3> Vuvw = MotorMath::InvclarkTransform(Vab);
	mMotorInfo.mDutyuvw = Vuvw;
}


void MotorCtrl::SVM(void) {
	mMotorInfo.mDutyuvw = MotorMath::SVM(mMotorInfo.mVab, mMotorInfo.mVoltageVCC);
}


void MotorCtrl::VoltageOutputTask(void) {
	//0~1のDutyをPWMで出力する。
	//0 <= mMotorInfo.mDuty <= 1
	TIMCtrl::floatDuty_ch1(mMotorInfo.mDutyuvw.at(0));
	TIMCtrl::floatDuty_ch2(mMotorInfo.mDutyuvw.at(1));
	TIMCtrl::floatDuty_ch3(mMotorInfo.mDutyuvw.at(2));
}


void MotorCtrl::ControlModeHandler() { //状態遷移を管理する関数
	float OpenLoopOmega = mArgCtrl.getArgOmega();
	float ObserverOmega = mObserver.GetEstOmegaE();

	float HFOmega = mSineWaveConvCalculator.GetEstOmegaE();

	switch(mControlMode) {
	case OpenLoop:
		if(OpenLoopOmega > OPENLOOP_END_OMEGA) {
			mControlMode = OpenLoopToFOC;
		}
		break;
	case OpenLoopToFOC:
		//OpenLoopへの遷移
		if(OpenLoopOmega < OPENLOOP_END_OMEGA) {
			mControlMode = OpenLoop;
		}
		//FOCへの遷移
		if(OPEN_TO_FOC_TRANSITION_COUNT_STEP2 < mTransitionCountForOpenToFOC2) {
			mControlMode = FOC; //何が起こるかわくわく　ここで本来書くべきではない。Handerが管理するべき。
			mTransitionCountForOpenToFOC = 0;
			mTransitionCountForOpenToFOC2 = 0;
		}
		break;
	case FOC:
		if(200 > ObserverOmega ){ //定常状態は400
			mControlMode = OpenLoopToFOC;
		}
		break;
	case FOC_Convolution:
		break;
	default:
		break;
	}


	//Debug or SPI
	if(mDEBUGorSPI == Debug) {//交互に入れ替える
		mDEBUGorSPI = SPI;
	} else {
		mDEBUGorSPI = Debug;
	}

}

void MotorCtrl::GPIODebugTask() {//Lチカでタイミングをオシロで見る
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//Board to Driver pin2
	asm("NOP");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}


//UICtrl
void MotorCtrl::BtnAct(void){//強制転流開始へのトリガ 割り込みから叩くためにここでラッパする
	if(mUIStatus.mStartStopTRG){
		BtnActOFF();
	} else {
		BtnActON();
	}
}


void MotorCtrl::BtnActOFF(void){//強制転流開始へのトリガOFF
	mUIStatus.mStartStopTRG = MotorStop;
	mControlMode = OpenLoop;
}


void MotorCtrl::BtnActON(void){//強制転流開始へのトリガON
	mUIStatus.mStartStopTRG = MotorStart;
}

