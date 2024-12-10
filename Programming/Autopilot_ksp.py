import math
import time
import krpc

turn_start_altitude = 250  # Начальная высота
turn_end_altitude = 45000  # Конечная высота
target_altitude = 181000  # Целевой апоапсис (высота орбиты)

conn = krpc.connect(name='Vostok 1')
vessel = conn.space_center.active_vessel  # Получение активной ракеты, которая будет управляться

# Получение текущего времени (UT), высоты, апоапсиса и количества топлива в твердотопливных ускорителях (SRB)
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
srb_fuel = conn.add_stream(stage_2_resources.amount, 'SolidFuel')

# Предворительная настройка
vessel.control.sas = False  # Отключается система автоматической стабилизации (SAS)
vessel.control.rcs = False  # Отключается реактивная система управления (RCS)
vessel.control.throttle = 1.0  # Устанавливается максимальная тяга (100% газа)

# Отсчет...
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')

# Активация первой стадии
vessel.control.activate_next_stage()  # Активируется первая ступень ракеты
vessel.auto_pilot.engage()  # Включается автопилот
vessel.auto_pilot.target_pitch_and_heading(90, 90)  # Задаётся начальный угол наклона ракеты (90 градусов, вертикальный взлёт) и курс (90 градусов, на север)

# Основная петля подъема
srbs_separated = False
turn_angle = 0
while True:

    # Гравитационный поворот
    # По мере набора высоты ракета постепенно изменяет угол наклона от вертикального (90 градусов)
    # до горизонтального (0 градусов), чтобы оптимизировать траекторию.
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    # Когда топливо в твердотопливных ускорителях заканчивается, они отделяются
    if not srbs_separated:
        if srb_fuel() < 0.1:
            vessel.control.activate_next_stage()
            srbs_separated = True
            print('SRBs separated')

    # Когда апоапсис приближается к целевому значению, тяга снижается, чтобы избежать перебора
    if apoapsis() > target_altitude*0.9:
        print('Approaching target apoapsis')
        break

# Тяга снижается до 25%, пока апоапсис не достигнет целевого значения.
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    pass
print('Target apoapsis reached')
# После достижения целевого апоапсиса двигатели полностью отключаются
vessel.control.throttle = 0.0

# Ракета продолжает лететь по инерции, пока не выйдет из атмосферы (высота 70,5 км)
print('Coasting out of atmosphere')
while altitude() < 70500:
    pass

# Рассчитывается необходимая скорость для циркуляризации орбиты (используется уравнение Вивиани).
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
# Создаётся узел манёвра для выполнения циркуляризации
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Рассчитывается время манёвра (используется уравнение ракеты)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Корабль ориентируется в направлении манёвра
print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Корабль ждёт, пока не наступит время манёвра
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

# Выполняется основной импульс для циркуляризации орбиты
print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(burn_time - 0.1)
print('Fine tuning')
# После основного импульса выполняется точная настройка
vessel.control.throttle = 0.05
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
while remaining_burn()[1] > 0:
    pass
vessel.control.throttle = 0.0
# Узел манёвра удаляется
node.remove()

print('Launch complete')
