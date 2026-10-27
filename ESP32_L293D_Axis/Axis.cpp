/*
 * Axis.cpp — реализация оси.
 *
 * Структура регулирования (как каскад ACOPOS):
 *   генератор профиля -> s_set, v_set
 *   контур положения:  v_cmd = v_set + position.kv * (s_set - s)
 *   контур скорости:   u = ff * v_set + speed.kv * (v_cmd - v) + I
 *   u (В) -> ШИМ на EN1 + направление IN1/IN2 микросхемы L293D
 */
#include "Axis.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#define AXIS_LEDC_CHANNEL 0   /* для ядра Arduino-ESP32 2.x */
#endif

/* ================= Энкодер (квадратурный x4 на прерываниях) ================= */

static volatile int32_t* s_encRaw = nullptr;
static uint32_t s_maskA = 0, s_maskB = 0;
static volatile uint8_t s_encPrev = 0;

/* индекс = (прошлое AB << 2) | текущее AB; A опережает B -> +1 */
static DRAM_ATTR const int8_t QDEC[16] = {
	 0, -1, +1,  0,
	+1,  0,  0, -1,
	-1,  0,  0, +1,
	 0, +1, -1,  0
};

static void IRAM_ATTR EncoderISR()
{
	uint32_t in = REG_READ(GPIO_IN_REG);
	uint8_t ab = ((in & s_maskA) ? 2 : 0) | ((in & s_maskB) ? 1 : 0);
	*s_encRaw += QDEC[(s_encPrev << 2) | ab];
	s_encPrev = ab;
}

/* ================= Выход на L293D ================= */

static void Drive_Write(Axis_typ* ax, float u, bool enable)
{
	uint32_t dutyMax = (1u << ax->drive.pwm_bits) - 1;
	uint32_t duty = 0;

	/* NaN/inf никогда не должны попасть в ШИМ: (uint32_t)NaN — неопределённое поведение */
	if (!isfinite(u)) enable = false;

	if (!enable) {
		digitalWrite(ax->drive.pin_in1, LOW);
		digitalWrite(ax->drive.pin_in2, LOW);
	} else {
		u *= ax->drive.dir;
		bool fwd = (u >= 0.0f);
		digitalWrite(ax->drive.pin_in1, fwd ? HIGH : LOW);
		digitalWrite(ax->drive.pin_in2, fwd ? LOW : HIGH);
		float k = fabsf(u) / ax->drive.u_supply;
		if (k > 1.0f) k = 1.0f;
		duty = (uint32_t)(k * dutyMax + 0.5f);
	}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
	ledcWrite(ax->drive.pin_en, duty);
#else
	ledcWrite(AXIS_LEDC_CHANNEL, duty);
#endif
}

/* ================= Вспомогательные ================= */

static float RampTo(float v, float vt, float acc, float dec, float dt)
{
	float dv = vt - v;
	bool accelerating = (v >= 0.0f && dv > 0.0f) || (v <= 0.0f && dv < 0.0f);
	float lim = (accelerating ? acc : dec) * dt;
	if (fabsf(dv) <= lim) return vt;
	return v + (dv > 0.0f ? lim : -lim);
}

static float Clamp(float x, float lim)
{
	if (x > lim) return lim;
	if (x < -lim) return -lim;
	return x;
}

static int32_t Encoder_Counts(Axis_typ* ax)
{
	return ax->encoder.dir * ax->encoder.raw;
}

static void Axis_SetError(Axis_typ* ax, uint16_t id)
{
	ax->message.error = true;
	ax->message.error_id = id;
	ax->controller.status = false;
	ax->move.mode = axMODE_STANDSTILL;
	ax->move.cmd_id++;
}

/* Сбросить задание на текущую фактическую позицию (без рывка при включении) */
static void Axis_SyncSetpoint(Axis_typ* ax)
{
	ax->monitor.s_set = ax->monitor.s;
	ax->monitor.v_set = 0.0f;
	ax->controller.speed.i_part = 0.0f;
	ax->move.mode = axMODE_STANDSTILL;
}

/* ================= Генератор профиля ================= */

static void Axis_Profile(Axis_typ* ax)
{
	AxisMove_typ* m = &ax->move;
	float dt = ax->dt;
	float s = ax->monitor.s_set;
	float v = ax->monitor.v_set;

	switch (m->mode) {
	case axMODE_STANDSTILL:
		v = 0.0f;
		break;

	case axMODE_VELOCITY:
		v = RampTo(v, m->target_v, m->acc, m->dec, dt);
		m->v_reached = (v == m->target_v);
		s += v * dt;
		break;

	case axMODE_STOPPING:
		v = RampTo(v, 0.0f, m->dec, m->dec, dt);
		s += v * dt;
		if (v == 0.0f) m->mode = axMODE_STANDSTILL;
		break;

	case axMODE_POSITION: {
		float dist = m->target_s - s;
		/* наибольшая скорость, с которой ещё можно остановиться в цели; дискретный аналог
		   sqrt(2*dec*dist) — с непрерывной формулой задание перелетает цель на ~1° */
		float h = m->dec * dt;
		float vStop = h * (sqrtf(0.25f + 2.0f * fabsf(dist) / (h * dt)) - 0.5f);
		float vDes = Clamp(dist >= 0.0f ? vStop : -vStop, m->v_max);
		v = RampTo(v, vDes, m->acc, m->dec, dt);
		float step = v * dt;
		/* фиксируем цель, только если скорость уже можно погасить за шаг; иначе
		   (команда пришла на ходу) проскакиваем цель и возвращаемся по профилю */
		bool slow = fabsf(v) <= 2.0f * m->dec * dt;
		if (slow && ((dist >= 0.0f && step >= dist) || (dist <= 0.0f && step <= dist))) {
			s = m->target_s;
			v = 0.0f;
			m->done = true;
			m->mode = axMODE_STANDSTILL;
		} else {
			s += step;
		}
		break;
	}
	}

	ax->monitor.s_set = s;
	ax->monitor.v_set = v;
}

/* ================= Публичные функции ================= */

void Axis_UpdateScaling(Axis_typ* ax)
{
	ax->encoder.counts_per_rev = ax->encoder.ppr * ax->encoder.mult * ax->encoder.gear_ratio;
	ax->encoder.units_per_count = ax->encoder.units_per_rev / ax->encoder.counts_per_rev;
}

void Axis_Init(Axis_typ* ax)
{
	Axis_UpdateScaling(ax);

	pinMode(ax->drive.pin_in1, OUTPUT);
	pinMode(ax->drive.pin_in2, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
	ledcAttach(ax->drive.pin_en, ax->drive.pwm_freq, ax->drive.pwm_bits);
#else
	ledcSetup(AXIS_LEDC_CHANNEL, ax->drive.pwm_freq, ax->drive.pwm_bits);
	ledcAttachPin(ax->drive.pin_en, AXIS_LEDC_CHANNEL);
#endif
	Drive_Write(ax, 0.0f, false);

	/* GPIO_IN_REG покрывает только GPIO0..31 */
	pinMode(ax->encoder.pin_a, INPUT_PULLUP);
	pinMode(ax->encoder.pin_b, INPUT_PULLUP);
	s_encRaw = &ax->encoder.raw;
	s_maskA = 1u << ax->encoder.pin_a;
	s_maskB = 1u << ax->encoder.pin_b;
	uint32_t in = REG_READ(GPIO_IN_REG);
	s_encPrev = ((in & s_maskA) ? 2 : 0) | ((in & s_maskB) ? 1 : 0);
	attachInterrupt(digitalPinToInterrupt(ax->encoder.pin_a), EncoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ax->encoder.pin_b), EncoderISR, CHANGE);

	ax->encoder.offset = Encoder_Counts(ax);
	ax->monitor.s = 0.0f;
	ax->monitor.v = 0.0f;
	ax->_s_prev = 0.0f;
	ax->controller.status = false;
	ax->message.error = false;
	ax->message.error_id = axERR_NONE;
	ax->move.cmd_id = 1;
	Axis_SyncSetpoint(ax);
}

void Axis_Cyclic(Axis_typ* ax)
{
	float dt = ax->dt;

	/* --- фактические значения --- */
	int32_t c = Encoder_Counts(ax);
	ax->monitor.counts = c;
	float s = (float)(c - ax->encoder.offset) * ax->encoder.units_per_count;
	float vRaw = (s - ax->_s_prev) / dt;
	ax->_s_prev = s;
	ax->monitor.s = s;
	ax->monitor.v += dt / (ax->controller.speed.t_filter + dt) * (vRaw - ax->monitor.v);

	if (!ax->controller.status) {
		Axis_SyncSetpoint(ax);
		ax->monitor.lag = 0.0f;
		ax->monitor.u = 0.0f;
		Drive_Write(ax, 0.0f, false);
		return;
	}

	/* --- задание --- */
	Axis_Profile(ax);

	/* --- контур положения --- */
	float lag = ax->monitor.s_set - s;
	ax->monitor.lag = lag;
	if (!(fabsf(lag) <= ax->limit.ds_stop)) {   /* так ловится и NaN */
		Axis_SetError(ax, axERR_LAG);
		ax->monitor.u = 0.0f;
		Drive_Write(ax, 0.0f, false);
		return;
	}
	float vCmd = ax->monitor.v_set + ax->controller.position.kv * lag;

	/* --- контур скорости (ПИ + прямая связь, антинасыщение) --- */
	AxisSpeedCtrl_typ* sc = &ax->controller.speed;
	float e = vCmd - ax->monitor.v;
	float uFF = sc->ff * ax->monitor.v_set;
	float uP = sc->kv * e;
	if (sc->tn > 0.0f) {
		float di = sc->kv * dt / sc->tn * e;
		float uTry = uFF + uP + sc->i_part + di;
		if (fabsf(uTry) < ax->drive.u_supply || uTry * di < 0.0f)
			sc->i_part += di;
	} else {
		sc->i_part = 0.0f;
	}
	float u = Clamp(uFF + uP + sc->i_part, ax->drive.u_supply);

	ax->monitor.u = u;
	Drive_Write(ax, u, true);
}

bool Axis_Power(Axis_typ* ax, bool on)
{
	if (on) {
		if (ax->message.error) return false;
		Axis_SyncSetpoint(ax);
		ax->controller.status = true;
	} else {
		ax->controller.status = false;
		ax->move.mode = axMODE_STANDSTILL;
	}
	ax->move.cmd_id++;
	return true;
}

void Axis_ErrorReset(Axis_typ* ax)
{
	ax->message.error = false;
	ax->message.error_id = axERR_NONE;
}

uint16_t Axis_SetPosition(Axis_typ* ax, float pos)
{
	if (ax->move.mode != axMODE_STANDSTILL) return axERR_NOT_STANDSTILL;
	if (!isfinite(pos)) return axERR_PARAM;
	int32_t c = Encoder_Counts(ax);
	ax->encoder.offset = c - (int32_t)lroundf(pos / ax->encoder.units_per_count);
	float s = (float)(c - ax->encoder.offset) * ax->encoder.units_per_count;
	ax->monitor.s = s;
	ax->_s_prev = s;
	ax->monitor.s_set = s;
	return axERR_NONE;
}

static uint16_t Axis_CheckMove(Axis_typ* ax, float acc, float dec)
{
	if (ax->message.error) return axERR_AXIS_ERROR;
	if (!ax->controller.status) return axERR_NOT_POWERED;
	if (!(acc > 0.0f) || !(dec > 0.0f)) return axERR_PARAM;   /* в т.ч. NaN */
	return axERR_NONE;
}

uint16_t Axis_MoveVelocity(Axis_typ* ax, float v, float acc, float dec, uint32_t* id)
{
	uint16_t err = Axis_CheckMove(ax, acc, dec);
	if (err) return err;
	if (!isfinite(v)) return axERR_PARAM;
	AxisMove_typ* m = &ax->move;
	m->target_v = Clamp(v, ax->limit.v);
	m->acc = fminf(acc, ax->limit.a);
	m->dec = fminf(dec, ax->limit.a);
	m->v_reached = false;
	m->done = false;
	m->mode = axMODE_VELOCITY;
	*id = ++m->cmd_id;
	return axERR_NONE;
}

uint16_t Axis_MoveAbsolute(Axis_typ* ax, float pos, float v, float acc, float dec, uint32_t* id)
{
	uint16_t err = Axis_CheckMove(ax, acc, dec);
	if (err) return err;
	if (!isfinite(pos) || !(v > 0.0f)) return axERR_PARAM;
	AxisMove_typ* m = &ax->move;
	m->target_s = pos;
	m->v_max = fminf(v, ax->limit.v);
	m->acc = fminf(acc, ax->limit.a);
	m->dec = fminf(dec, ax->limit.a);
	m->done = false;
	m->v_reached = false;
	m->mode = axMODE_POSITION;
	*id = ++m->cmd_id;
	return axERR_NONE;
}

uint32_t Axis_Stop(Axis_typ* ax, float dec)
{
	AxisMove_typ* m = &ax->move;
	m->dec = (dec > 0.0f) ? fminf(dec, ax->limit.a) : ax->limit.a;
	if (m->mode != axMODE_STANDSTILL) m->mode = axMODE_STOPPING;
	return ++m->cmd_id;
}
