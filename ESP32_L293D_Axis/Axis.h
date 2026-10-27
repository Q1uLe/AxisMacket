/*
 * Axis.h — "привод" оси: энкодер, генератор профиля, каскадный регулятор, ШИМ на L293D.
 * Аналог встроенного ПО ACOPOS: ФБ только отдают команды, всё движение считается здесь.
 */
#pragma once
#include "AxisTypes.h"

void     Axis_Init(Axis_typ* ax);          /* вызвать один раз после заполнения параметров */
void     Axis_UpdateScaling(Axis_typ* ax); /* пересчёт масштаба после смены ppr/gear_ratio */
void     Axis_Cyclic(Axis_typ* ax);        /* вызывать каждые ax->dt секунд */

bool     Axis_Power(Axis_typ* ax, bool on);
void     Axis_ErrorReset(Axis_typ* ax);
uint16_t Axis_SetPosition(Axis_typ* ax, float pos);  /* аналог MC_Home direct */

/* Команды движения. Возвращают код ошибки (axERR_NONE при успехе), id — номер команды. */
uint16_t Axis_MoveVelocity(Axis_typ* ax, float v, float acc, float dec, uint32_t* id);
uint16_t Axis_MoveAbsolute(Axis_typ* ax, float pos, float v, float acc, float dec, uint32_t* id);
uint32_t Axis_Stop(Axis_typ* ax, float dec);
