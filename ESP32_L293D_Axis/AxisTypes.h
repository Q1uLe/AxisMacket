/*
 * AxisTypes.h — типы данных оси (аналог Global.typ / ACP10AXIS_typ в Automation Studio)
 *
 * Вся информация об оси собрана в одной глобальной структуре gAxis, как объект оси
 * ACOPOS: параметры привода, энкодера, регулятора, ограничения, монитор и сообщения.
 */
#pragma once
#include <Arduino.h>

/* ---------- Перечисления (как в ACP10_MC) ---------- */

typedef enum : uint8_t {
	mcPOSITIVE_DIR = 0,
	mcNEGATIVE_DIR = 1
} McDirectionEnum;

typedef enum : uint8_t {
	axMODE_STANDSTILL = 0,  /* стоим, удерживаем s_set */
	axMODE_VELOCITY,        /* непрерывное движение с заданной скоростью */
	axMODE_POSITION,        /* движение в абсолютную позицию */
	axMODE_STOPPING         /* торможение до нуля */
} AxisMoveMode;

typedef enum : uint16_t {
	axERR_NONE        = 0,
	axERR_NO_AXIS     = 1000, /* у ФБ не задан указатель Axis */
	axERR_NOT_POWERED = 1001, /* ось не включена (controller.status = 0) */
	axERR_LAG         = 1002, /* превышена ошибка рассогласования limit.ds_stop */
	axERR_PARAM       = 1003, /* неверный параметр (ускорение <= 0 и т.п.) */
	axERR_AXIS_ERROR  = 1004, /* ось в ошибке, нужен сброс */
	axERR_NOT_STANDSTILL = 1005 /* операция допустима только в покое */
} AxisErrorId;

/* ---------- Подструктуры ---------- */

typedef struct {
	int8_t   pin_en;     /* L293D EN1 (ШИМ) */
	int8_t   pin_in1;    /* L293D IN1 */
	int8_t   pin_in2;    /* L293D IN2 */
	uint32_t pwm_freq;   /* Гц */
	uint8_t  pwm_bits;   /* разрядность ШИМ */
	int8_t   dir;        /* +1 / -1 — инверсия направления двигателя */
	float    u_supply;   /* напряжение питания двигателя, В */
} AxisDrive_typ;

typedef struct {
	int8_t  pin_a;           /* канал A (жёлтый) */
	int8_t  pin_b;           /* канал B (зелёный) */
	float   ppr;             /* импульсов на оборот вала ДВИГАТЕЛЯ на канал (11) */
	uint8_t mult;            /* кратность декодирования (4 = x4) */
	float   gear_ratio;      /* передаточное число редуктора */
	float   units_per_rev;   /* единиц на оборот выходного вала (360 = градусы) */
	int8_t  dir;             /* +1 / -1 — инверсия счёта */
	/* вычисляемые (Axis_UpdateScaling) */
	float   counts_per_rev;  /* инкрементов на оборот выходного вала */
	float   units_per_count; /* единиц на инкремент */
	/* рабочие */
	volatile int32_t raw;    /* счётчик, пишется только из прерывания */
	int32_t offset;          /* смещение нуля (Axis_SetPosition) */
} AxisEncoder_typ;

typedef struct {
	float kv;        /* пропорциональный коэффициент, В / (ед/с) */
	float tn;        /* время интегрирования, с (0 — интегратор выключен) */
	float ff;        /* прямая связь по скорости, В / (ед/с) */
	float t_filter;  /* постоянная фильтра фактической скорости, с */
	float i_part;    /* текущее значение интегральной части, В */
} AxisSpeedCtrl_typ;

typedef struct {
	float kv;        /* коэффициент контура положения, 1/с */
} AxisPosCtrl_typ;

typedef struct {
	bool status;     /* регулятор включён (аналог controller.status в ACP10) */
	AxisPosCtrl_typ   position;
	AxisSpeedCtrl_typ speed;
} AxisController_typ;

typedef struct {
	float v;              /* макс. скорость, ед/с */
	float a;              /* макс. ускорение/замедление, ед/с^2 */
	float ds_stop;        /* предел ошибки рассогласования, ед */
	float in_pos_window;  /* окно "в позиции", ед */
	float in_vel_window;  /* окно "на скорости", ед/с */
} AxisLimit_typ;

typedef struct {
	float   s;        /* фактическая позиция, ед */
	float   v;        /* фактическая скорость (фильтр.), ед/с */
	float   s_set;    /* заданная позиция (выход генератора профиля), ед */
	float   v_set;    /* заданная скорость, ед/с */
	float   lag;      /* рассогласование s_set - s, ед */
	float   u;        /* напряжение на двигателе, В */
	int32_t counts;   /* инкременты энкодера с учётом dir */
} AxisMonitor_typ;

typedef struct {
	AxisMoveMode mode;
	uint32_t cmd_id;     /* номер текущей команды; меняется — значит предыдущая прервана */
	float target_s;
	float target_v;
	float v_max;
	float acc;
	float dec;
	bool  done;          /* позиционирование завершено (генератор дошёл до цели) */
	bool  v_reached;     /* генератор вышел на target_v */
} AxisMove_typ;

typedef struct {
	bool     error;
	uint16_t error_id;
} AxisMessage_typ;

/* ---------- Объект оси ---------- */

typedef struct {
	float dt;                       /* период цикла, с */
	AxisDrive_typ      drive;
	AxisEncoder_typ    encoder;
	AxisController_typ controller;
	AxisLimit_typ      limit;
	AxisMonitor_typ    monitor;
	AxisMove_typ       move;
	AxisMessage_typ    message;
	float _s_prev;                  /* внутреннее: позиция прошлого цикла */
} Axis_typ;

extern Axis_typ gAxis;
