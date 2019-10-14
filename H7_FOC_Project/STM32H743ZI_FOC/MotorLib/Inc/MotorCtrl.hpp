/*
 * MotorCtrl.hpp
 *
 *  Created on: Aug 5, 2019
 *      Author: watashi
 */

#ifndef MOTORCTRL_HPP_
#define MOTORCTRL_HPP_

#include "paramsetting.h" //パラメータのマクロ
#include "MotorLibDefPack.hpp"

// ../SystemLib/Inc/
#include "STM32SystemPack.h"

#include "ADCCtrl.hpp"
#include "TIMCtrl.hpp"

//#include "GPIOInit.hpp"
//#include "USARTInit.hpp"


#include "ArgCtrl.hpp"
#include "MotorMath.hpp"
#include "PID.hpp"
#include "Observer.hpp"
#include <array>

#include "PWM.hpp"
//#include "UART.hpp"

#include "DebugCtrl.hpp"

#include "SEGGER_RTT.h"

class MotorCtrl {
public:

	class MotorInfo {//モータの情報を格納するClass
	public:
		std::array<float, 3> mIuvw;//単位[A]
		std::array<float, 2> mIab;//単位[A]
		std::array<float, 2> mIdq;//単位[A]
		std::array<float, 2> mIgd; //単位[A] ganmadelta

		std::array<float, 3> mVuvw;//単位[V]
		std::array<float, 2> mVab;//単位[V]
		std::array<float, 2> mVdq;//単位[V]
		std::array<float, 2> mVgd; //単位[V] ganmadelta

		std::array<float, 2> mIgdTarget;//
		std::array<float, 2> mIgdErr;
		float mVoltageVCC;
		std::array<float, 3> mDutyuvw;//単位[%]
		fp_rad mdqArg;//単位[rad]
		fp_rad mgdArg;//単位[rad]
		fp_rad mArgErr;//単位[rad]
	};

	int mDebugC=0; //SEGGERおためしでつくった

	enum MotorStartStopStatus {//ON OFFのフラグ管理
			MotorStop = 0,
			MotorStart = 1,
	};

	enum OperationMode { //測定か運転か
		Measure = -1,
		Drive = 1,
	};

	enum ControlMode { //強制転流、FOCなどの動作モード管理
		OpenLoop = 1,
		OpenLoopToFOC = 2,
		FOC = 3,
	};

	class UIStatus {
	public:
		MotorStartStopStatus mStartStopTRG;
	};

private:
	MotorInfo mMotorInfo;
	Observer mObserver;

	PID mIdPID, mIqPID;
	PID mIganmaPID, mIdeltaPID;

	ArgCtrl mArgCtrl;

	OperationMode mOperationMode;
	ControlMode mControlMode;

	UIStatus mUIStatus;

	DebugCtrl mDebug;

public:
	MotorCtrl();
	virtual ~MotorCtrl();
	void SetMotorInfo(MotorInfo pMotorInfo);

	void InitSystem(void);
	void InitMotorControl(void);
	void InitObserver(void);

	void HighFreqTask(void);

	void MotorDrive(void);

	void MotorOutputTask(void);
	void MotorOutputTaskSVM(void);

	void ControlModeHandler();

	void setIuvw(float pIu, float pIv, float pIw);

	void ReadAngle(void);

	void clarkTransform(void);
	void parkTransform(void);
	void parkGanmaDelta(void);
	void parkabtogd(void);

	std::array<float, 2> getIdq(void);
	std::array<float, 2> getIgd(void);

	void PIDdq_control(std::array<float, 2> pErrIdq);//ここでPID使う？？ライブラリインクルード必要だよね？
	void PIDgd_control(std::array<float, 2> pErrIgd);//どちらかが死にclassになるけど毎回呼ぶ作業でif文使いたくない。

	void setVdq(std::array<float, 2> pVdq);//強制転流用
	void setVgd(std::array<float, 2> pVgd);

	void invParkgdtoab(void);
	void invParkGanmaDelta(void);
	void invParkTransform(void);
	void invClarkTransform(void);
	void SVM(void);

	//Task
	void ReadCurrentTask();
	void ReadVoltageTask();
	void AngleTaskForOpenLoop();
	void AngleTaskForFOC();
	void ObserverTask();
	void CurrentPITask();

	void GPIODebugTask();

	//Debug
	void JLinkDebug();

	//UI
	void BtnAct(void);
	void BtnActOFF(void);
	void BtnActON(void);

	//for debug
	void DbgUart(std::string pStr);
	void DebugTask(float pIu, float pIv, float pIw, float pArg);

private:
	int mlogcount = 0;
	int mTransitionCountForOpenToFOC = 0;
};

#endif /* MOTORCTRL_HPP_ */
