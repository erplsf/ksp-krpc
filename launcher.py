import math
import time

import krpc

conn = krpc.connect(name='Launcher')
vessel = conn.space_center.active_vessel
flight = vessel.flight()
kerbin_flight = vessel.flight(vessel.orbit.body.reference_frame)

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

vessel.auto_pilot.target_heading = 90
vessel.auto_pilot.target_pitch = 90
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()

target_apoapsis = 90_000

altitude = conn.add_stream(getattr, flight, 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
pitch = conn.add_stream(getattr, flight, 'pitch')
current_stage = conn.add_stream(getattr, vessel.control, 'current_stage')
ut = conn.add_stream(getattr, conn.space_center, 'ut')


def pf(altitude):
    pitch_angle = 90 * (1 - (altitude / 70_000) ** 0.5)

    if altitude < 0:
        return float(90)
    elif pitch_angle < 0:
        return float(0)
    else:
        return pitch_angle


def current_decouple_stage():
    return current_stage() - 1


def error(value, target_value):
    return 1 - (value / target_value)


resources_in_current_stage = conn.add_stream(vessel.resources_in_decouple_stage(stage=current_decouple_stage(), cumulative=False).amount, 'LiquidFuel')
ascended = False

while True:
    while not ascended:
        target_pitch = pf(altitude())
        if abs(error(pitch(), target_pitch)) > 0.01:
            vessel.auto_pilot.target_pitch = target_pitch

        if resources_in_current_stage() < 0.1:
            vessel.control.activate_next_stage()
            time.sleep(1)

        if error(apoapsis(), target_apoapsis) < 0.01:
            ascended = True

    print("Target apoapsis reached.")
    vessel.control.throttle = 0

    # Plan circularization burn (using vis-viva equation)
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a_per = vessel.orbit.semi_major_axis
    a_apo = r
    v_per = math.sqrt(mu*((2./r)-(1./a_per)))
    v_apo = math.sqrt(mu*((2./r)-(1./a_apo)))
    delta_v = v_apo - v_per
    print(f"dv: {delta_v}")

    node = vessel.control.add_node(
        ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Orientate ship
    print('Orientating ship for circularization burn')
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.wait()

    # Wait until burn
    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    print('Ready to execute burn')
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burn_time/2.) > 0:
        pass

    print('Executing burn')
    vessel.control.throttle = 1.0
    time.sleep(burn_time - 0.1)
    print('Fine tuning')
    vessel.control.throttle = 0.05
    remaining_burn = conn.add_stream(
        node.remaining_burn_vector, node.reference_frame)
    while remaining_burn()[1] > 0:
        pass
    vessel.control.throttle = 0.0
    node.remove()

    print('Launch complete')
