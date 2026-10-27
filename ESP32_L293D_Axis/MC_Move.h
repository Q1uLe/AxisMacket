/*
 * MC_Move.h — функциональные блоки движения в стиле PLCopen / ACP10_MC.
 *
 * Использование как в Automation Studio (C):
 *     fbMoveAbsolute.Axis     = &gAxis;
 *     fbMoveAbsolute.Position = 90.0f;
 *     fbMoveAbsolute.Execute  = true;
 *     MC_MoveAbsolute(&fbMoveAbsolute);   // вызывать каждый цикл
 */
#pragma once
#include "AxisTypes.h"

/* ФБ 1 — непрерывное движение с заданной скоростью */
typedef struct {
	/* VAR_INPUT */
	Axis_typ*       Axis;
	bool            Execute;       /* фронт — старт; спад — торможение до остановки */
	float           Velocity;      /* ед/с (модуль), можно менять на ходу */
	float           Acceleration;  /* ед/с^2 */
	float           Deceleration;  /* ед/с^2 */
	McDirectionEnum Direction;
	/* VAR_OUTPUT */
	bool     InVelocity;
	bool     Busy;
	bool     Active;
	bool     CommandAborted;
	bool     Error;
	uint16_t ErrorID;
	/* внутренние */
	bool     _execOld;
	uint32_t _id;
	float    _vOld;
	McDirectionEnum _dirOld;
} MC_MoveVelocity_typ;

/* ФБ 2 — перемещение в абсолютную позицию по энкодеру */
typedef struct {
	/* VAR_INPUT */
	Axis_typ* Axis;
	bool      Execute;       /* фронт — старт */
	float     Position;      /* ед */
	float     Velocity;      /* ед/с */
	float     Acceleration;  /* ед/с^2 */
	float     Deceleration;  /* ед/с^2 */
	/* VAR_OUTPUT */
	bool     Done;
	bool     Busy;
	bool     Active;
	bool     CommandAborted;
	bool     Error;
	uint16_t ErrorID;
	/* внутренние */
	bool     _execOld;
	uint32_t _id;
} MC_MoveAbsolute_typ;

void MC_MoveVelocity(MC_MoveVelocity_typ* inst);
void MC_MoveAbsolute(MC_MoveAbsolute_typ* inst);
