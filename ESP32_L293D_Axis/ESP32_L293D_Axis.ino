/*
 * ESP32 + L293D + JGA25-370 (12 В, 130 об/мин, энкодер 11 имп/об)
 * Управление осью по образцу ACOPOS / Automation Studio:
 *   gAxis            — глобальный объект оси (AxisTypes.h)
 *   MC_MoveVelocity  — ФБ 1, непрерывное движение
 *   MC_MoveAbsolute  — ФБ 2, перемещение в позицию по 2-канальному энкодеру
 *
 * ProgramInit / ProgramCyclic — как _INIT / _CYCLIC в AS. ProgramCyclic выполняется
 * в отдельной задаче FreeRTOS с периодом 5 мс (аналог циклического класса задач).
 * loop() — только консоль Serial (аналог Watch-окна): команды попадают в очередь
 * и применяются внутри цикла, поэтому к ФБ и gAxis обращается одна задача.
 *
 * Единицы оси: градусы выходного вала, °/с, °/с².  Команды — "help" в мониторе порта, 115200.
 */
#include "AxisTypes.h"
#include "Axis.h"
#include "MC_Move.h"

/* ===================== Конфигурация ===================== */

#define CYCLE_MS        5

#define PIN_L293_EN1    25   /* ШИМ */
#define PIN_L293_IN1    26
#define PIN_L293_IN2    27
#define PIN_ENC_A       18   /* жёлтый провод */
#define PIN_ENC_B       19   /* зелёный провод */

#define MOTOR_SUPPLY_V  12.0f
#define ENC_PPR         11.0f    /* импульсов на оборот вала двигателя на канал */
#define GEAR_RATIO      46.8f    /* 130 об/мин -> см. README, уточняется командой cal */

/* прерывание энкодера читает GPIO_IN_REG, а он покрывает только GPIO0..31 */
static_assert(PIN_ENC_A < 32 && PIN_ENC_B < 32, "энкодер должен быть на GPIO0..31");

/* ===================== Глобальные переменные (Global.var) ===================== */

Axis_typ gAxis;

MC_MoveVelocity_typ fbMoveVelocity;
MC_MoveAbsolute_typ fbMoveAbsolute;

volatile uint32_t gCycleTimeUs = 0;
volatile uint32_t gCycleTimeMaxUs = 0;

/* ===================== Обмен консоль <-> цикл ===================== */

typedef enum : uint8_t {
	CMD_POWER_ON, CMD_POWER_OFF, CMD_RESET, CMD_HOME,
	CMD_MOVE_VEL, CMD_MOVE_ABS,
	CMD_SET_ACC, CMD_SET_VEL, CMD_SET_PARAM,
	CMD_CAL_START, CMD_CAL_END
} CmdOp;

typedef enum : uint8_t {
	PAR_KV, PAR_TN, PAR_FF, PAR_KPOS, PAR_TF, PAR_RATIO, PAR_PPR,
	PAR_ENCDIR, PAR_MOTDIR, PAR_VMAX, PAR_AMAX, PAR_LAG, PAR_WIN
} ParamId;

typedef struct { CmdOp op; uint8_t par; float a; float b; } Cmd;

typedef enum : uint8_t {
	EVT_ABS_DONE, EVT_ABS_ABORTED, EVT_ABS_ERROR,
	EVT_VEL_INVEL, EVT_VEL_ABORTED, EVT_VEL_ERROR,
	EVT_AXIS_ERROR, EVT_REJECTED, EVT_CAL_DONE, EVT_INFO_OK
} EvtCode;

typedef struct { EvtCode code; uint16_t id; float val; } Evt;

static QueueHandle_t cmdQueue;
static QueueHandle_t evtQueue;

static void PostEvent(EvtCode code, uint16_t id, float val)
{
	Evt e = { code, id, val };
	xQueueSend(evtQueue, &e, 0);
}

/* ===================== Прикладная программа ===================== */

static bool    restartVel = false, restartAbs = false;
static float   defaultVel = 360.0f;   /* скорость для MoveAbsolute, °/с */
static float   defaultAcc = 1500.0f;  /* °/с² */
static int32_t calStartCounts = 0;
static bool    calActive = false;

static void ProgramInit()
{
	gAxis.dt = CYCLE_MS / 1000.0f;

	gAxis.drive.pin_en   = PIN_L293_EN1;
	gAxis.drive.pin_in1  = PIN_L293_IN1;
	gAxis.drive.pin_in2  = PIN_L293_IN2;
	gAxis.drive.pwm_freq = 5000;
	gAxis.drive.pwm_bits = 10;
	gAxis.drive.dir      = +1;
	gAxis.drive.u_supply = MOTOR_SUPPLY_V;

	gAxis.encoder.pin_a         = PIN_ENC_A;
	gAxis.encoder.pin_b         = PIN_ENC_B;
	gAxis.encoder.ppr           = ENC_PPR;
	gAxis.encoder.mult          = 4;
	gAxis.encoder.gear_ratio    = GEAR_RATIO;
	gAxis.encoder.units_per_rev = 360.0f;
	gAxis.encoder.dir           = +1;

	gAxis.controller.position.kv    = 15.0f;    /* 1/с */
	gAxis.controller.speed.kv       = 0.015f;   /* В/(°/с) */
	gAxis.controller.speed.tn       = 0.08f;    /* с */
	gAxis.controller.speed.ff       = MOTOR_SUPPLY_V / 780.0f;  /* 12 В -> 130 об/мин = 780 °/с */
	gAxis.controller.speed.t_filter = 0.02f;    /* с */

	gAxis.limit.v             = 600.0f;   /* °/с (100 об/мин, запас на падение в L293D) */
	gAxis.limit.a             = 5000.0f;  /* °/с² */
	gAxis.limit.ds_stop       = 90.0f;    /* ° */
	gAxis.limit.in_pos_window = 1.0f;     /* ° */
	gAxis.limit.in_vel_window = 30.0f;    /* °/с */

	Axis_Init(&gAxis);

	fbMoveVelocity.Axis         = &gAxis;
	fbMoveVelocity.Acceleration = defaultAcc;
	fbMoveVelocity.Deceleration = defaultAcc;
	fbMoveAbsolute.Axis         = &gAxis;
	fbMoveAbsolute.Velocity     = defaultVel;
	fbMoveAbsolute.Acceleration = defaultAcc;
	fbMoveAbsolute.Deceleration = defaultAcc;
}

static void SetParam(ParamId p, float x)
{
	/* усиления, фильтр, время интегрирования — не отрицательные; пределы и масштаб — строго > 0 */
	bool nonNeg = (p == PAR_KV || p == PAR_TN || p == PAR_FF || p == PAR_KPOS || p == PAR_TF);
	bool sign   = (p == PAR_ENCDIR || p == PAR_MOTDIR);
	if (!sign && (nonNeg ? !(x >= 0.0f) : !(x > 0.0f))) {
		PostEvent(EVT_REJECTED, axERR_PARAM, 0.0f);
		return;
	}

	switch (p) {
	case PAR_KV:     gAxis.controller.speed.kv = x; break;
	case PAR_TN:     gAxis.controller.speed.tn = x; gAxis.controller.speed.i_part = 0; break;
	case PAR_FF:     gAxis.controller.speed.ff = x; break;
	case PAR_KPOS:   gAxis.controller.position.kv = x; break;
	case PAR_TF:     gAxis.controller.speed.t_filter = x; break;
	case PAR_VMAX:   gAxis.limit.v = x; break;
	case PAR_AMAX:   gAxis.limit.a = x; break;
	case PAR_LAG:    gAxis.limit.ds_stop = x; break;
	case PAR_WIN:    gAxis.limit.in_pos_window = x; break;
	case PAR_MOTDIR:
	case PAR_RATIO:
	case PAR_PPR:
	case PAR_ENCDIR:
		/* смена направления/масштаба на ходу = положительная обратная связь — только при выключенной оси */
		if (gAxis.controller.status) { PostEvent(EVT_REJECTED, axERR_NOT_STANDSTILL, 0.0f); return; }
		if (p == PAR_MOTDIR) { gAxis.drive.dir = (x < 0) ? -1 : 1; break; }
		if (p == PAR_RATIO) gAxis.encoder.gear_ratio = x;
		if (p == PAR_PPR)   gAxis.encoder.ppr = x;
		if (p == PAR_ENCDIR) gAxis.encoder.dir = (x < 0) ? -1 : 1;
		Axis_UpdateScaling(&gAxis);
		Axis_SetPosition(&gAxis, 0.0f);
		calActive = false;
		break;
	}
	PostEvent(EVT_INFO_OK, 0, 0.0f);
}

static void ApplyCommand(const Cmd& c)
{
	switch (c.op) {
	case CMD_POWER_ON:
		if (!Axis_Power(&gAxis, true)) PostEvent(EVT_REJECTED, axERR_AXIS_ERROR, 0.0f);
		else PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	case CMD_POWER_OFF:
		Axis_Power(&gAxis, false);
		PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	case CMD_RESET:
		Axis_ErrorReset(&gAxis);
		PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	case CMD_HOME: {
		uint16_t err = Axis_SetPosition(&gAxis, c.a);
		if (err) PostEvent(EVT_REJECTED, err, 0.0f); else PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	}
	case CMD_MOVE_VEL:
		fbMoveVelocity.Velocity  = fabsf(c.a);
		fbMoveVelocity.Direction = (c.a < 0) ? mcNEGATIVE_DIR : mcPOSITIVE_DIR;
		if (!fbMoveVelocity.Execute) fbMoveVelocity.Execute = true;
		else if (!fbMoveVelocity.Active) { fbMoveVelocity.Execute = false; restartVel = true; }
		/* если блок активен — скорость обновится на ходу */
		break;
	case CMD_MOVE_ABS:
		fbMoveAbsolute.Position = c.a;
		fbMoveAbsolute.Velocity = (c.b > 0) ? c.b : defaultVel;
		if (!fbMoveAbsolute.Execute) fbMoveAbsolute.Execute = true;
		else { fbMoveAbsolute.Execute = false; restartAbs = true; }
		break;
	case CMD_SET_ACC:
		defaultAcc = c.a;
		fbMoveVelocity.Acceleration = fbMoveVelocity.Deceleration = c.a;
		fbMoveAbsolute.Acceleration = fbMoveAbsolute.Deceleration = c.a;
		PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	case CMD_SET_VEL:
		defaultVel = c.a;
		PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	case CMD_SET_PARAM:
		SetParam((ParamId)c.par, c.a);
		break;
	case CMD_CAL_START:
		Axis_Power(&gAxis, false);
		calStartCounts = gAxis.monitor.counts;
		calActive = true;
		PostEvent(EVT_INFO_OK, 0, 0.0f);
		break;
	case CMD_CAL_END: {
		if (!calActive || gAxis.controller.status || !(c.a > 0)) { PostEvent(EVT_REJECTED, axERR_PARAM, 0.0f); break; }
		calActive = false;
		float n = fabsf((float)(gAxis.monitor.counts - calStartCounts));
		float ratio = n / (c.a * gAxis.encoder.ppr * gAxis.encoder.mult);
		if (ratio < 1.0f) { PostEvent(EVT_REJECTED, axERR_PARAM, 0.0f); break; }
		gAxis.encoder.gear_ratio = ratio;
		Axis_UpdateScaling(&gAxis);
		Axis_SetPosition(&gAxis, 0.0f);
		PostEvent(EVT_CAL_DONE, 0, ratio);
		break;
	}
	}
}

static void ProgramCyclic()
{
	/* повторный фронт Execute, запрошенный в прошлом цикле */
	if (restartVel) { fbMoveVelocity.Execute = true; restartVel = false; }
	if (restartAbs) { fbMoveAbsolute.Execute = true; restartAbs = false; }

	Cmd c;
	while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) ApplyCommand(c);

	/* снять Execute по завершении — как шаговая логика программы в AS */
	if (fbMoveAbsolute.Execute && !fbMoveAbsolute.Busy &&
	    (fbMoveAbsolute.Done || fbMoveAbsolute.Error || fbMoveAbsolute.CommandAborted) && !restartAbs)
		fbMoveAbsolute.Execute = false;
	if (fbMoveVelocity.Execute && (fbMoveVelocity.Error || fbMoveVelocity.CommandAborted) && !restartVel)
		fbMoveVelocity.Execute = false;

	/* --- вызовы ФБ --- */
	bool absDone = fbMoveAbsolute.Done, absAb = fbMoveAbsolute.CommandAborted, absErr = fbMoveAbsolute.Error;
	bool velIn = fbMoveVelocity.InVelocity, velAb = fbMoveVelocity.CommandAborted, velErr = fbMoveVelocity.Error;
	bool axErr = gAxis.message.error;

	MC_MoveVelocity(&fbMoveVelocity);
	MC_MoveAbsolute(&fbMoveAbsolute);

	/* --- ось: энкодер, профиль, регулятор, ШИМ --- */
	Axis_Cyclic(&gAxis);

	/* --- события для консоли (по фронтам выходов) --- */
	if (fbMoveAbsolute.Done && !absDone) PostEvent(EVT_ABS_DONE, 0, gAxis.monitor.s);
	if (fbMoveAbsolute.CommandAborted && !absAb) PostEvent(EVT_ABS_ABORTED, 0, 0.0f);
	if (fbMoveAbsolute.Error && !absErr) PostEvent(EVT_ABS_ERROR, fbMoveAbsolute.ErrorID, 0.0f);
	if (fbMoveVelocity.InVelocity && !velIn) PostEvent(EVT_VEL_INVEL, 0, gAxis.monitor.v);
	if (fbMoveVelocity.CommandAborted && !velAb) PostEvent(EVT_VEL_ABORTED, 0, 0.0f);
	if (fbMoveVelocity.Error && !velErr) PostEvent(EVT_VEL_ERROR, fbMoveVelocity.ErrorID, 0.0f);
	if (gAxis.message.error && !axErr) PostEvent(EVT_AXIS_ERROR, gAxis.message.error_id, 0.0f);
}

static void CyclicTask(void*)
{
	TickType_t last = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&last, pdMS_TO_TICKS(CYCLE_MS));
		uint32_t t0 = micros();
		ProgramCyclic();
		uint32_t t = micros() - t0;
		gCycleTimeUs = t;
		if (t > gCycleTimeMaxUs) gCycleTimeMaxUs = t;
	}
}

/* ===================== Консоль ===================== */

static bool monitorOn = false;

static const char* ErrorText(uint16_t id)
{
	switch (id) {
	case axERR_NO_AXIS:        return "не задан Axis";
	case axERR_NOT_POWERED:    return "ось не включена (on)";
	case axERR_LAG:            return "превышено рассогласование (заклинило / неверное направление энкодера)";
	case axERR_PARAM:          return "неверный параметр";
	case axERR_AXIS_ERROR:     return "ось в ошибке (reset)";
	case axERR_NOT_STANDSTILL: return "допустимо только в покое / при выключенной оси";
	default:                   return "";
	}
}

static void PrintHelp()
{
	Serial.println(F(
		"\n--- Ось ESP32 + L293D + JGA25-370 ---\n"
		"on / off            включить / выключить регулятор оси\n"
		"reset               сброс ошибки оси\n"
		"home [pos]          задать текущую позицию (по умолчанию 0)\n"
		"mv <v>              MC_MoveVelocity, v в °/с (знак = направление)\n"
		"rpm <n>             MC_MoveVelocity, n в об/мин выходного вала\n"
		"stop                торможение (MC_MoveVelocity с v = 0)\n"
		"ma <pos> [v]        MC_MoveAbsolute в позицию pos, °\n"
		"acc <a>             ускорение/замедление для ФБ, °/с²\n"
		"vel <v>             скорость по умолчанию для ma, °/с\n"
		"st                  состояние оси и ФБ\n"
		"par                 параметры оси\n"
		"mon [0|1]           поток s_set,s,v_set,v,u для Serial Plotter\n"
		"set <имя> <знач>    kv tn ff kpos tf vmax amax lag win motdir encdir ratio ppr\n"
		"cal                 калибровка: ось выкл., отметить вал, повернуть рукой N оборотов\n"
		"cal <N>             ...затем ввести N — посчитает передаточное число"));
}

static void PrintStatus()
{
	Serial.printf("Ось: %s%s  режим=%d  cmd_id=%lu\n",
		gAxis.controller.status ? "ВКЛ" : "выкл",
		gAxis.message.error ? "  ОШИБКА" : "", gAxis.move.mode, (unsigned long)gAxis.move.cmd_id);
	if (gAxis.message.error)
		Serial.printf("  ошибка %u: %s\n", gAxis.message.error_id, ErrorText(gAxis.message.error_id));
	Serial.printf("  s=%.2f°  s_set=%.2f°  lag=%.2f°\n", gAxis.monitor.s, gAxis.monitor.s_set, gAxis.monitor.lag);
	Serial.printf("  v=%.1f°/с (%.1f об/мин)  v_set=%.1f°/с  u=%.2f В  counts=%ld\n",
		gAxis.monitor.v, gAxis.monitor.v / 6.0f, gAxis.monitor.v_set, gAxis.monitor.u, (long)gAxis.monitor.counts);
	Serial.printf("MC_MoveVelocity: Exec=%d Busy=%d Active=%d InVel=%d Abort=%d Err=%d(%u)\n",
		fbMoveVelocity.Execute, fbMoveVelocity.Busy, fbMoveVelocity.Active, fbMoveVelocity.InVelocity,
		fbMoveVelocity.CommandAborted, fbMoveVelocity.Error, fbMoveVelocity.ErrorID);
	Serial.printf("MC_MoveAbsolute: Exec=%d Busy=%d Active=%d Done=%d Abort=%d Err=%d(%u)  Pos=%.2f\n",
		fbMoveAbsolute.Execute, fbMoveAbsolute.Busy, fbMoveAbsolute.Active, fbMoveAbsolute.Done,
		fbMoveAbsolute.CommandAborted, fbMoveAbsolute.Error, fbMoveAbsolute.ErrorID, fbMoveAbsolute.Position);
	Serial.printf("Цикл: %lu мкс (макс %lu мкс) из %d мс\n",
		(unsigned long)gCycleTimeUs, (unsigned long)gCycleTimeMaxUs, CYCLE_MS);
}

static void PrintParams()
{
	Serial.printf("Энкодер: ppr=%.1f x%u, i=%.2f, %.1f инкр/об вых. вала, %.4f °/инкр, dir=%d\n",
		gAxis.encoder.ppr, gAxis.encoder.mult, gAxis.encoder.gear_ratio,
		gAxis.encoder.counts_per_rev, gAxis.encoder.units_per_count, gAxis.encoder.dir);
	Serial.printf("Привод: U=%.1f В, ШИМ %lu Гц/%u бит, dir=%d\n",
		gAxis.drive.u_supply, (unsigned long)gAxis.drive.pwm_freq, gAxis.drive.pwm_bits, gAxis.drive.dir);
	Serial.printf("Регулятор: kpos=%.2f 1/с | kv=%.4f В/(°/с) tn=%.3f с ff=%.4f tf=%.3f с\n",
		gAxis.controller.position.kv, gAxis.controller.speed.kv, gAxis.controller.speed.tn,
		gAxis.controller.speed.ff, gAxis.controller.speed.t_filter);
	Serial.printf("Ограничения: v=%.0f °/с a=%.0f °/с² ds_stop=%.1f° окно=%.2f°\n",
		gAxis.limit.v, gAxis.limit.a, gAxis.limit.ds_stop, gAxis.limit.in_pos_window);
	Serial.printf("ФБ: acc=%.0f °/с², vel(ma)=%.0f °/с\n", defaultAcc, defaultVel);
}

static void PrintEvents()
{
	Evt e;
	while (xQueueReceive(evtQueue, &e, 0) == pdTRUE) {
		switch (e.code) {
		case EVT_ABS_DONE:    Serial.printf("MC_MoveAbsolute: Done, s=%.2f°\n", e.val); break;
		case EVT_ABS_ABORTED: Serial.println("MC_MoveAbsolute: CommandAborted"); break;
		case EVT_ABS_ERROR:   Serial.printf("MC_MoveAbsolute: Error %u — %s\n", e.id, ErrorText(e.id)); break;
		case EVT_VEL_INVEL:   Serial.printf("MC_MoveVelocity: InVelocity, v=%.1f°/с\n", e.val); break;
		case EVT_VEL_ABORTED: Serial.println("MC_MoveVelocity: CommandAborted"); break;
		case EVT_VEL_ERROR:   Serial.printf("MC_MoveVelocity: Error %u — %s\n", e.id, ErrorText(e.id)); break;
		case EVT_AXIS_ERROR:  Serial.printf("ОСЬ: ошибка %u — %s\n", e.id, ErrorText(e.id)); break;
		case EVT_REJECTED:    Serial.printf("Отклонено: %s\n", ErrorText(e.id)); break;
		case EVT_CAL_DONE:
			Serial.printf("Калибровка: i = %.2f (%.1f инкр/об). Впишите GEAR_RATIO в скетч.\n",
				e.val, gAxis.encoder.counts_per_rev);
			break;
		case EVT_INFO_OK:     Serial.println("ok"); break;
		}
	}
}

static bool ParseParam(const char* name, ParamId* p)
{
	static const struct { const char* n; ParamId p; } tbl[] = {
		{"kv", PAR_KV}, {"tn", PAR_TN}, {"ff", PAR_FF}, {"kpos", PAR_KPOS}, {"tf", PAR_TF},
		{"ratio", PAR_RATIO}, {"ppr", PAR_PPR}, {"encdir", PAR_ENCDIR}, {"motdir", PAR_MOTDIR},
		{"vmax", PAR_VMAX}, {"amax", PAR_AMAX}, {"lag", PAR_LAG}, {"win", PAR_WIN}
	};
	for (auto& t : tbl) if (strcmp(name, t.n) == 0) { *p = t.p; return true; }
	return false;
}

static void SendCmd(CmdOp op, float a, float b, uint8_t par)
{
	Cmd c = { op, par, a, b };
	if (xQueueSend(cmdQueue, &c, pdMS_TO_TICKS(20)) != pdTRUE)
		Serial.println("очередь команд занята, повторите");
}

/* Число целиком и конечное: "12.5" — да; "12abc", "nan", "inf", "" — нет.
   atof() тут нельзя: "nan" из консоли прошёл бы мимо защиты по рассогласованию. */
static bool ParseNum(const char* s, float* out)
{
	if (!s) return false;
	char* end;
	float x = strtof(s, &end);
	if (end == s || *end != '\0' || !isfinite(x)) return false;
	*out = x;
	return true;
}

static void HandleLine(char* line)
{
	char* cmd = strtok(line, " \t");
	if (!cmd) return;
	char* a1 = strtok(nullptr, " \t");
	char* a2 = strtok(nullptr, " \t");
	for (char* p = cmd; *p; ++p) *p = tolower((unsigned char)*p);

	float x = 0.0f, y = 0.0f;
	bool n1 = ParseNum(a1, &x);
	bool n2 = ParseNum(a2, &y);
	bool badArg = (a1 && !n1) || (a2 && !n2);

	if      (!strcmp(cmd, "help") || !strcmp(cmd, "?")) PrintHelp();
	else if (!strcmp(cmd, "st"))    PrintStatus();
	else if (!strcmp(cmd, "par"))   PrintParams();
	else if (!strcmp(cmd, "on"))    SendCmd(CMD_POWER_ON, 0.0f, 0.0f, 0);
	else if (!strcmp(cmd, "off"))   SendCmd(CMD_POWER_OFF, 0.0f, 0.0f, 0);
	else if (!strcmp(cmd, "reset")) SendCmd(CMD_RESET, 0.0f, 0.0f, 0);
	else if (!strcmp(cmd, "stop"))  SendCmd(CMD_MOVE_VEL, 0.0f, 0.0f, 0);
	else if (!strcmp(cmd, "mon"))   monitorOn = n1 ? (x != 0.0f) : !monitorOn;
	else if (!strcmp(cmd, "set")) {
		ParamId p;
		if (!a1 || !ParseParam(a1, &p)) Serial.println("неизвестный параметр");
		else if (!n2) Serial.println("нужно число: set <имя> <значение>");
		else SendCmd(CMD_SET_PARAM, y, 0.0f, p);
	}
	else if (badArg) Serial.println("аргумент должен быть конечным числом");
	else if (!strcmp(cmd, "home"))  SendCmd(CMD_HOME, x, 0.0f, 0);
	else if (!strcmp(cmd, "mv") && n1)  SendCmd(CMD_MOVE_VEL, x, 0.0f, 0);
	else if (!strcmp(cmd, "rpm") && n1) SendCmd(CMD_MOVE_VEL, x * gAxis.encoder.units_per_rev / 60.0f, 0.0f, 0);
	else if (!strcmp(cmd, "ma") && n1)  SendCmd(CMD_MOVE_ABS, x, y, 0);
	else if (!strcmp(cmd, "acc") && n1 && x > 0) SendCmd(CMD_SET_ACC, x, 0.0f, 0);
	else if (!strcmp(cmd, "vel") && n1 && x > 0) SendCmd(CMD_SET_VEL, x, 0.0f, 0);
	else if (!strcmp(cmd, "cal")) {
		if (n1) SendCmd(CMD_CAL_END, x, 0.0f, 0);
		else {
			SendCmd(CMD_CAL_START, 0.0f, 0.0f, 0);
			Serial.println("Ось выключена. Отметьте положение выходного вала, поверните его рукой "
			               "ровно N оборотов и введите: cal N");
		}
	}
	else Serial.println("? (help — список команд)");
}

static void ReadConsole()
{
	static char buf[64];
	static uint8_t len = 0;
	while (Serial.available()) {
		char ch = Serial.read();
		if (ch == '\r' || ch == '\n') {
			if (len) { buf[len] = 0; HandleLine(buf); len = 0; }
		} else if (len < sizeof(buf) - 1) {
			buf[len++] = ch;
		}
	}
}

/* ===================== setup / loop ===================== */

void setup()
{
	Serial.begin(115200);
	delay(200);

	cmdQueue = xQueueCreate(8, sizeof(Cmd));
	evtQueue = xQueueCreate(16, sizeof(Evt));

	ProgramInit();
	xTaskCreatePinnedToCore(CyclicTask, "Cyclic#1", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);

	PrintHelp();
	PrintParams();
}

void loop()
{
	ReadConsole();
	PrintEvents();

	static uint32_t tMon = 0;
	if (monitorOn && millis() - tMon >= 20) {
		tMon = millis();
		Serial.printf("s_set:%.2f,s:%.2f,v_set:%.1f,v:%.1f,u:%.2f\n",
			gAxis.monitor.s_set, gAxis.monitor.s, gAxis.monitor.v_set, gAxis.monitor.v, gAxis.monitor.u);
	}
	delay(1);
}
