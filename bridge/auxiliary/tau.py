"""
Динамические звенья и регуляторы
"""

import math
from enum import Enum
from typing import Optional

from bridge import const
from bridge.auxiliary import aux


class FOD:
    """
    Реальное дифференцирующее звено первого порядка
    """

    def __init__(self, T: float, Ts: float, is_angle: bool = False) -> None:
        """
        Конструктор

        T - постоянная времени ФНЧ
        dT - период квантования
        """
        self._t = T
        self._ts = Ts
        self._int = 0.0
        self._out = 0.0
        self._is_angle = is_angle

    def process(self, x: float) -> float:
        """
        Рассчитать и получить следующее значение выхода звена

        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТОВАНИЯ

        x - новое значение входа
        """
        err = x - self._int
        if self._is_angle:
            if err > math.pi:
                err -= 2 * math.pi
                self._int += 2 * math.pi
            elif err < -math.pi:
                err += 2 * math.pi
                self._int -= 2 * math.pi
        self._out = err / self._t
        self._int += self._out * self._ts
        return self._out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out


class FOLP:
    """
    Фильтр низких частот первого порядка
    """

    def __init__(self, T: float, Ts: float) -> None:
        """
        Конструктор

        T - постоянная времени ФНЧ
        dT - период квантования
        """
        self._t = T
        self._ts = Ts
        self._int = 0.0
        self._out = 0.0

    def process(self, x: float) -> float:
        """
        Рассчитать и получить следующее значение выхода звена

        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТОВАНИЯ

        x - новое значение входа
        """
        err = x - self._out
        self._int += err * self._ts
        self._out = self._int / self._t
        return self._out

    def process_(self, x: float, dT: float) -> float:
        """
        Рассчитать и получить следующее значение выхода звена

        x - новое значение входа
        """
        err = x - self._out
        self._int += err * dT
        self._out = self._int / self._t
        self._out = self._int / math.pow(self._t, dT / self._t)  # NOTE
        return self._out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out


class Mode(Enum):
    """
    Названия наборов коэффициентов регулятора
    """

    NORMAL = 0
    CATCH = 1


class AdaptivePDController:
    """
    Пропорционально-скользяще-интегральный регулятор

    (В отличие от ПИД берёт производную от скорости изменения регулируемой
    величины, а не ошибки)
    """

    def __init__(
        self,
        gain: list[float],
        kd: list[float],
        kpp: list[float],
    ) -> None:
        """
        Конструктор

        каждый параметр - список коэффициентов для разных режимов
        gain - коэффициент усиления регулятора (П составляющая)
        kd - коэффициент дифференциальной части (типа Д составляющая)
        ki - коэффициент интегрирующей части (И составляющая)
        max_out - Максимальное значение управляющего воздействия
        """
        self.__gain = gain
        self.__kd = kd
        self.__kpp = kpp
        self.__out = 0.0
        self.__mode = Mode.NORMAL

    def select_mode(self, mode: Mode) -> None:
        """
        Выбрать набор коэффициентов регулятора
        """
        if self.__mode != mode:
            self.__mode = mode

    def __get_gains(self) -> tuple[float, float, float]:
        """
        Получить коэффициенты регулятора
        """
        return (
            self.__gain[self.__mode.value],
            self.__kd[self.__mode.value],
            self.__kpp[self.__mode.value],
        )

    def process(self, xerr: float, x_i: float, total_dist: Optional[float] = None) -> float:
        """
        Рассчитать следующий тик регулятора
        """
        gain, k_d, k_pp = self.__get_gains()

        dist_for_gain = total_dist if total_dist is not None else abs(xerr)

        effective_gain = gain
        mult_th = 500.0

        multiplier = 1.0 + k_pp * (1.0 - aux.minmax((mult_th - dist_for_gain) / mult_th, 0, 1))
        effective_gain *= multiplier

        s = xerr + k_d * x_i
        u = effective_gain * s

        self.__out = u

        return self.__out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self.__out


class PDController:
    """
    Классический пропорционально-дифференциальный (ПД) регулятор.
    """

    def __init__(
        self,
        gain: list[float],
        kd: list[float],
    ) -> None:
        self.__gain = gain
        self.__kd = kd
        self.__out = 0.0
        self.__mode = Mode.NORMAL

    def select_mode(self, mode: Mode) -> None:
        """Выбрать набор коэффициентов регулятора"""
        if self.__mode != mode:
            self.__mode = mode

    def __get_gains(self) -> tuple[float, float]:
        """Получить коэффициенты регулятора"""
        return (
            self.__gain[self.__mode.value],
            self.__kd[self.__mode.value],
        )

    def process(self, xerr: float, x_i: float, total_dist: Optional[float] = None) -> float:
        """
        Рассчитать следующий тик классического ПД-регулятора.
        total_dist принимается для API-совместимости, но математически
        линейному регулятору проекция не нужна.
        """
        gain, k_d = self.__get_gains()

        u = gain * xerr + k_d * x_i

        u = aux.minmax(u, const.MAX_SPEED_R)

        self.__out = u
        return self.__out

    def get_val(self) -> float:
        """Получить последнее значение выхода звена без расчета"""
        return self.__out


class SqrtController:
    """
    Пропорционально-скользяще-интегральный регулятор

    (В отличие от ПИД берёт производную от скорости изменения регулируемой
    величины, а не ошибки)
    """

    def __init__(
        self,
        kd: list[float],
        accl: list[float],
    ) -> None:
        """
        Конструктор

        каждый параметр - список коэффициентов для разных режимов
        gain - коэффициент усиления регулятора (П составляющая)
        kd - коэффициент дифференциальной части (типа Д составляющая)
        ki - коэффициент интегрирующей части (И составляющая)
        max_out - Максимальное значение управляющего воздействия
        """
        self.__kd = kd
        self.__accl = accl
        self.__out = 0.0
        self.__mode = Mode.NORMAL

    def select_mode(self, mode: Mode) -> None:
        """
        Выбрать набор коэффициентов регулятора
        """
        if self.__mode != mode:
            self.__mode = mode

    def __get_gains(self) -> tuple[float, float]:
        """
        Получить коэффициенты регулятора
        """
        return (self.__kd[self.__mode.value], self.__accl[self.__mode.value])

    def process(self, xerr: float, x_i: float, total_dist: Optional[float] = None) -> float:
        """
        Рассчитать следующий тик регулятора
        """
        k_d, accl = self.__get_gains()

        a_max = accl
        d_err = total_dist if total_dist is not None else abs(xerr)
        threshold = 1000.0

        if d_err < threshold:
            dynamic_gain = math.sqrt(2 * a_max * threshold) / threshold
            u = dynamic_gain * xerr + k_d * x_i
        else:
            v_total = math.sqrt(2 * a_max * d_err)
            v_total = min(v_total, const.MAX_SPEED)

            if total_dist is not None and total_dist > 0:
                u = v_total * (xerr / total_dist) + k_d * x_i
            else:
                u = math.copysign(v_total, xerr) + k_d * x_i

        self.__out = u

        return self.__out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self.__out
