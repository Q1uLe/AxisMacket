/*
 * MC_Move.cpp — реализация ФБ MC_MoveVelocity и MC_MoveAbsolute.
 *
 * ФБ не считают движение сами: по фронту Execute они отдают команду оси (Axis.cpp),
 * затем каждый цикл следят за её состоянием. Если ось получила другую команду
 * (move.cmd_id изменился), блок выставляет CommandAborted — как в ACP10_MC.
 */
#include "MC_Move.h"
#include "Axis.h"

/* Общая проверка активной команды: true — команда ещё наша и ось в порядке */
static bool CheckAxis(Axis_typ* ax, uint32_t id, bool* error, uint16_t* errorId, bool* aborted)
{
	if (ax->message.error) {
		*error = true;
		*errorId = ax->message.error_id;
		return false;
	}
	if (!ax->controller.status) {
		*error = true;
		*errorId = axERR_NOT_POWERED;
		return false;
	}
	if (ax->move.cmd_id != id) {
		*aborted = true;
		return false;
	}
	return true;
}

/* ===================== MC_MoveVelocity ===================== */

static float SignedVelocity(const MC_MoveVelocity_typ* f)
{
	float v = fabsf(f->Velocity);
	return (f->Direction == mcNEGATIVE_DIR) ? -v : v;
}

void MC_MoveVelocity(MC_MoveVelocity_typ* f)
{
	bool rising  = f->Execute && !f->_execOld;
	bool falling = !f->Execute && f->_execOld;
	f->_execOld = f->Execute;

	/* выходы держатся, пока Execute = 1; после спада — сбрасываются через цикл */
	if (!f->Execute && !f->Busy) {
		f->InVelocity = false;
		f->CommandAborted = false;
		f->Error = false;
		f->ErrorID = axERR_NONE;
	}

	if (f->Axis == nullptr) {
		f->Error = true;
		f->ErrorID = axERR_NO_AXIS;
		return;
	}
	Axis_typ* ax = f->Axis;

	if (rising) {
		f->InVelocity = false;
		f->CommandAborted = false;
		f->Error = false;
		f->ErrorID = axERR_NONE;
		uint16_t err = Axis_MoveVelocity(ax, SignedVelocity(f), f->Acceleration, f->Deceleration, &f->_id);
		if (err) {
			f->Error = true;
			f->ErrorID = err;
			f->Busy = f->Active = false;
			return;
		}
		f->_vOld = f->Velocity;
		f->_dirOld = f->Direction;
		f->Busy = f->Active = true;
		return;
	}

	if (!f->Busy) return;

	if (!CheckAxis(ax, f->_id, &f->Error, &f->ErrorID, &f->CommandAborted)) {
		f->Busy = f->Active = f->InVelocity = false;
		return;
	}

	if (falling) {
		Axis_Stop(ax, f->Deceleration);
		f->Busy = f->Active = f->InVelocity = false;
		return;
	}

	/* смена скорости/направления на ходу, без повторного фронта */
	if (f->Velocity != f->_vOld || f->Direction != f->_dirOld) {
		uint16_t err = Axis_MoveVelocity(ax, SignedVelocity(f), f->Acceleration, f->Deceleration, &f->_id);
		if (err) {
			f->Error = true;
			f->ErrorID = err;
			f->Busy = f->Active = f->InVelocity = false;
			return;
		}
		f->_vOld = f->Velocity;
		f->_dirOld = f->Direction;
	}

	f->InVelocity = ax->move.v_reached &&
	                fabsf(ax->monitor.v - ax->monitor.v_set) <= ax->limit.in_vel_window;
}

/* ===================== MC_MoveAbsolute ===================== */

void MC_MoveAbsolute(MC_MoveAbsolute_typ* f)
{
	bool rising = f->Execute && !f->_execOld;
	f->_execOld = f->Execute;

	if (!f->Execute && !f->Busy) {
		f->Done = false;
		f->CommandAborted = false;
		f->Error = false;
		f->ErrorID = axERR_NONE;
	}

	if (f->Axis == nullptr) {
		f->Error = true;
		f->ErrorID = axERR_NO_AXIS;
		return;
	}
	Axis_typ* ax = f->Axis;

	if (rising) {
		f->Done = false;
		f->CommandAborted = false;
		f->Error = false;
		f->ErrorID = axERR_NONE;
		uint16_t err = Axis_MoveAbsolute(ax, f->Position, f->Velocity, f->Acceleration, f->Deceleration, &f->_id);
		if (err) {
			f->Error = true;
			f->ErrorID = err;
			f->Busy = f->Active = false;
			return;
		}
		f->Busy = f->Active = true;
		return;
	}

	if (!f->Busy) return;

	/* спад Execute движение не прерывает (как в PLCopen) */
	if (!CheckAxis(ax, f->_id, &f->Error, &f->ErrorID, &f->CommandAborted)) {
		f->Busy = f->Active = false;
		return;
	}

	/* Done: генератор дошёл до цели и фактическая позиция в окне */
	if (ax->move.done && fabsf(ax->monitor.s - ax->move.target_s) <= ax->limit.in_pos_window) {
		f->Done = true;
		f->Busy = f->Active = false;
	}
}
