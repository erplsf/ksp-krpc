import krpc
import time
import math

conn = krpc.connect(name='Launcher')
vessel = conn.space_center.active_vessel
flight = vessel.flight()
kerbin_flight = vessel.flight(vessel.orbit.body.reference_frame)


def pf(altitude):
    pitch_angle = 90 * (1 - (altitude / 70_000) ** 0.5)

    if altitude < 0:
        return float(90)
    elif pitch_angle < 0:
        return float(0)
    else:
        return pitch_angle


vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

vessel.auto_pilot.target_heading = 90
vessel.auto_pilot.target_pitch = 90
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()

target_apoapsis = 90_000
atmosphere_edge = 70_000
a = pitch_factor(atmosphere_edge)
target_apo_reached = False

s3_resources_amount = conn.add_stream(vessel.resources_in_decouple_stage(stage=3, cumulative=False).amount, 'LiquidFuel')
s3_separated = False

altitude = conn.add_stream(getattr, flight, 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
pitch = conn.add_stream(getattr, flight, 'pitch')
current_stage = conn.add_stream(getattr, vessel.control, 'current_stage')


def current_decouple_stage():
    return current_stage() - 1

resources_in_current_stage = conn.add_stream(vessel.resources_in_decouple_stage(stage=current_decouple_stage(), cumulative=False).amount, 'LiquidFuel')


def error(value, target_value):
    return 1 - (value / target_value)


while True:
    # print(f"alt: {altitude()}")
    target_pitch = pf(altitude())
    # print(f"tp: {target_pitch}")
    # print(f"cp: {pitch()}")
    if abs(error(pitch(), target_pitch)) > 0.01:
        vessel.auto_pilot.target_pitch = target_pitch

    # print(f"cs: {current_stage()}")
    # print(f"r: {resources_in_current_stage()}")
    if resources_in_current_stage() < 0.1:
        vessel.control.activate_next_stage()
        time.sleep(1)

    # print(f"apo: {apoapsis()}")
    if error(apoapsis(), target_apoapsis) < 0.1:
        print("Program finished, target apoapsis reached.")
        vessel.control.throttle = 0
        vessel.auto_pilot.disengage()
        break
